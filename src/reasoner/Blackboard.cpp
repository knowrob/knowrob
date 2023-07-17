/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/Logger.h"
#include "knowrob/reasoner/BuiltinEvaluator.h"
#include "knowrob/reasoner/Blackboard.h"
#include "knowrob/formulas/Conjunction.h"
#include "knowrob/formulas/Disjunction.h"
#include "knowrob/queries/QueryError.h"

#include <memory>

using namespace knowrob;

Blackboard::Blackboard(
		ReasonerManager *reasonerManager,
		const std::shared_ptr<AnswerQueue> &outputQueue,
		const std::shared_ptr<const Query> &goal)
		: reasonerManager_(reasonerManager),
		  builtinEvaluator_(std::make_shared<DefinedReasoner>("builtins", BuiltinEvaluator::get())),
		  outputQueue_(outputQueue),
		  goal_(goal)
{
	// decompose the reasoning task
	inputStream_ = std::make_shared<AnswerBroadcaster>();
	inputChannel_ = AnswerStream::Channel::create(inputStream_);
	outBroadcaster_ = std::make_shared<AnswerBroadcaster>();

	// create a channel of outputQueue_, and add it as subscriber
	auto queueChannel = AnswerStream::Channel::create(outputQueue_);
	outBroadcaster_->addSubscriber(queueChannel);

	createReasoningPipeline(inputStream_, outBroadcaster_);
}

Blackboard::~Blackboard()
{
	stop();
}

void Blackboard::start()
{
	// push empty message into in stream, and close the channel
	inputChannel_->push(AnswerStream::bos());
	inputChannel_->close();
}

void Blackboard::stop()
{
	for(auto &inputStream : reasonerInputs_) {
		inputStream->stop();
	}
}

ReasoningGraph Blackboard::decomposeFormula(const std::shared_ptr<Formula> &phi) const //NOLINT
{
	switch(phi->type()) {
		case FormulaType::CONJUNCTION: {
			auto args = ((Conjunction*)phi.get())->formulae();
			auto graph(decomposeFormula(args[0]));
			for(int i=1; i<args.size(); ++i) {
				graph.conjunction(decomposeFormula(args[i]));
			}
			return graph;
		}
		case FormulaType::DISJUNCTION: {
			auto args = ((Disjunction*)phi.get())->formulae();
			auto graph(decomposeFormula(args[0]));
			for(int i=1; i<args.size(); ++i) {
				graph.disjunction(decomposeFormula(args[i]));
			}
			return graph;
		}
		case FormulaType::PREDICATE: {
			return decomposePredicate(std::static_pointer_cast<Predicate>(phi));
		}
	}
}

ReasoningGraph Blackboard::decomposePredicate(const std::shared_ptr<Predicate> &phi) const
{
	// get ensemble of reasoner
	auto predicateDescription = reasonerManager_->getPredicateDefinition(
			phi->indicator());
	if (!predicateDescription || predicateDescription->reasonerEnsemble().empty()) {
		throw QueryError("no reasoner registered for query `{}`", *phi);
	}

	ReasoningGraph out; {
		if(predicateDescription->predicateType() == PredicateType::BUILT_IN) {
			// builtins are not evaluated wrt. to the database, all options
			// to evaluate one can be collapsed into a single node.
			// FIXME: introduce a notion of meta predicates, they are currently
			//   seen as builtins here, and it is ignored if their higher-order argument
			//   is a database predicate or not, which may cause incomplete results.
			//   for a meta predicate with database predicate each option of evaluation
			//   should be added instead.
			auto newNode = std::make_shared<ReasoningGraph::Node>(
					phi, predicateDescription->reasonerEnsemble(),
					predicateDescription->predicateType());
			out.addInitialNode(newNode);
		}
		else {
			for(auto &r : predicateDescription->reasonerEnsemble()) {
				auto newNode = std::make_shared<ReasoningGraph::Node>(
						phi, r, predicateDescription->predicateType());
				out.addInitialNode(newNode);
			}
		}
	}
	return out;
}

void Blackboard::createReasoningPipeline(const std::shared_ptr<AnswerBroadcaster> &inputStream,
										 const std::shared_ptr<AnswerBroadcaster> &outputStream)
{
	auto graph = decomposeFormula(goal_->formula());
	for(auto &n0 : graph.initialNodes())
		createReasoningPipeline(inputStream, outputStream, n0);
}

void Blackboard::createReasoningPipeline( //NOLINT
		const std::shared_ptr<AnswerBroadcaster> &pipelineInput,
		const std::shared_ptr<AnswerBroadcaster> &pipelineOutput,
		const std::shared_ptr<ReasoningGraph::Node> &n0)
{
	// create a subquery for the predicate p
	std::shared_ptr<Query> subQuery = std::make_shared<Query>(n0->phi());
	// create an output stream for this node
	std::shared_ptr<AnswerBroadcaster> nodeOutput = (n0->successors().empty() ?
                                                     pipelineOutput : std::make_shared<AnswerBroadcaster>());

	auto n0_reasoner = *n0->reasonerAlternatives().begin();
	if(n0->predicateType() == PredicateType::BUILT_IN) {
		// prefer to use BuiltinEvaluator
		auto p = ((Predicate*)n0->phi().get());
		if(BuiltinEvaluator::get()->isBuiltinSupported(p->indicator())) {
			n0_reasoner = builtinEvaluator_;
		}
	}
	createReasoningStep(n0_reasoner, subQuery, pipelineInput, nodeOutput);

	// continue for successors
	for(auto &successor : n0->successors()) {
		createReasoningPipeline(nodeOutput, pipelineOutput, successor);
	}
}

void Blackboard::createReasoningStep(const std::shared_ptr<DefinedReasoner> &r,
									 const std::shared_ptr<Query> &subQuery,
									 const std::shared_ptr<AnswerBroadcaster> &stepInput,
									 const std::shared_ptr<AnswerBroadcaster> &stepOutput)
{
	std::shared_ptr<Blackboard::Stream> reasonerIn;
	std::shared_ptr<AnswerStream::Channel> reasonerInChan, reasonerOutChan;
	// create IO channels
	reasonerOutChan = AnswerStream::Channel::create(stepOutput);
	reasonerIn = std::make_shared<Blackboard::Stream>(
			r,reasonerOutChan, subQuery);
	reasonerInChan = AnswerStream::Channel::create(reasonerIn);
	stepInput->addSubscriber(reasonerInChan);
	// create a new segment
	reasonerInputs_.push_back(reasonerIn);
}


Blackboard::Stream::Stream(
	const std::shared_ptr<DefinedReasoner> &reasoner,
	const std::shared_ptr<AnswerStream::Channel> &outputStream,
	const std::shared_ptr<Query> &goal)
: AnswerStream(),
  queryID_(reinterpret_cast<std::uintptr_t>(this)),
  reasoner_(reasoner),
  outputStream_(outputStream),
  goal_(goal),
  isQueryOpened_(true),
  hasStopRequest_(false)
{
	// tell the reasoner that a new request has been made
	// FIXME: remove blackboard class
	//(*reasoner_)()->startQuery(queryID_, goal);
}

Blackboard::Stream::~Stream()
{
	stop();
}

void Blackboard::Stream::push(const AnswerPtr &msg)
{
	if(AnswerStream::isEOS(msg)) {
		// tell the reasoner to finish up.
		// if hasStopRequest_=true it means that the reasoner is requested
		// to immediately shutdown. however, note that not all reasoner
		// implementations may support this and may take longer to stop anyway.
		if(isQueryOpened()) {
			isQueryOpened_ = false;
			// FIXME: need to call finishQuery if exception happens in runner!!
			//          else consumer will never be called with EOS
            // FIXME: remove blackboard class
			//(*reasoner_)()->finishQuery(queryID_, outputStream_, hasStopRequest());
		}
	}
	else if(!isQueryOpened()) {
		KB_WARN("ignoring attempt to write to a closed stream.");
	}
	else {
		// evaluate an instance of the input query
		auto queryInstance = std::make_shared<AllocatedQuery>(
				goal_, outputStream_, msg);
        // FIXME: remove blackboard class
		//(*reasoner_)()->runQueryInstance(queryID_, queryInstance);
	}
}

void Blackboard::Stream::stop()
{
	hasStopRequest_ = true;
	close();
}
