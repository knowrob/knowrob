/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/logging.h>
#include <knowrob/builtins.h>
#include <knowrob/Blackboard.h>

#include <memory>

using namespace knowrob;


template<class T> std::shared_ptr<T> createConnectiveFormula(
		const FormulaPtr &phi1, const FormulaPtr &phi2, FormulaType type)
{
	// attempt to merge if phi1 or phi2 is already a connective formula of the given type
	if(phi1->type() == type) {
		auto *c1 = (T*)phi1.get();
		if(phi2->type() == type) {
			auto *c2 = (T*)phi2.get();
			std::vector<FormulaPtr> args(c1->formulae().size()+c2->formulae().size());
			for(int i=0; i<c1->formulae().size(); i++) args[i] = c1->formulae()[i];
			for(int i=0; i<c2->formulae().size(); i++) args[i+c1->formulae().size()] = c2->formulae()[i];
			return std::make_shared<T>(args);
		}
		else {
			std::vector<FormulaPtr> args(c1->formulae().size()+1);
			for(int i=0; i<c1->formulae().size(); i++) args[i] = c1->formulae()[i];
			args[c1->formulae().size()] = phi2;
			return std::make_shared<T>(args);
		}
	}
	else if(phi2->type() == type) {
		auto *c2 = (T*)phi2.get();
		std::vector<FormulaPtr> args(c2->formulae().size()+1);
		args[0] = phi1;
		for(int i=0; i<c2->formulae().size(); i++) args[i+1] = c2->formulae()[i];
		return std::make_shared<T>(args);
	}
	else {
		return std::make_shared<T>(T({phi1, phi2}));
	}
}


ReasoningGraph::Node::Node(
		const std::shared_ptr<Formula> &phi,
		const std::shared_ptr<ManagedReasoner> &reasoner,
		PredicateType predicateType)
: phi_(phi),
  predicateType_(predicateType),
  reasonerChoices_({ reasoner })
{
}

ReasoningGraph::Node::Node(
		const std::shared_ptr<Formula> &phi,
		const std::set<std::shared_ptr<ManagedReasoner>> &reasonerChoices,
		PredicateType predicateType)
: phi_(phi),
  predicateType_(predicateType),
  reasonerChoices_(reasonerChoices)
{
}

bool ReasoningGraph::Node::canBeCombinedWith(
		const NodePtr &other, ReasonerCapability requiredCapability)
{
	for(auto &r1 : reasonerChoices_) {
		// reasoner must support the capability
		// and other node must have the same reasoner choice
		if(r1->reasoner()->hasCapability(requiredCapability) &&
		   other->reasonerChoices_.find(r1) != other->reasonerChoices_.end())
			return true;
	}
	return false;
}

bool ReasoningGraph::Node::isBuiltinSupported(const PredicateFormula &phi) const
{
	for(auto &r1 : reasonerChoices_) {
		if(r1->reasoner()->getPredicateDescription(phi.predicate()->indicator()) != nullptr)
			return true;
	}
	return false;
}


void ReasoningGraph::addInitialNode(const NodePtr &node)
{
	initialNodes_.push_back(node);
	if(node->successors_.empty()) {
		terminalNodes_.push_back(node);
	}
}

void ReasoningGraph::removeNode(const NodePtr &node)
{
	if(node->predecessors_.empty()) {
		initialNodes_.remove(node);
	}
	else {
		for(auto &predecessor : node->predecessors_) {
			predecessor->successors_.remove(node);
			if(node->successors_.empty()) terminalNodes_.push_back(predecessor);
		}
		node->predecessors_.clear();
	}

	if(node->successors_.empty()) {
		terminalNodes_.remove(node);
	}
	else {
		for(auto &successor : node->successors_) {
			successor->predecessors_.remove(node);
			if(node->predecessors_.empty()) initialNodes_.push_back(successor);
		}
		node->successors_.clear();
	}
}

void ReasoningGraph::addSuccessor(const NodePtr &predecessor, const NodePtr &successor)
{
	if(predecessor->successors_.empty()) terminalNodes_.remove(predecessor);
	if(successor->predecessors_.empty()) initialNodes_.remove(successor);

	predecessor->successors_.push_back(successor);
	successor->predecessors_.push_back(predecessor);
}

void ReasoningGraph::disjunction(const ReasoningGraph &other)
{
	// gather initial nodes of this graph without successors
	std::list<NodePtr> simpleNodes;
	for(auto &node1 : initialNodes_) {
		if(node1->successors_.empty()) simpleNodes.push_back(node1);
	}
	// iterate over initial nodes of the other graph.
	// if the node has no successors it may be combinable into a disjunctive query
	// with a node in simpleNodes.
	for(auto &node2 : other.initialNodes())
	{
		if(node2->successors_.empty()) {
			bool hasNode = false;
			for(auto &simple1 : simpleNodes)
			{
				if(simple1->canBeCombinedWith(node2, CAPABILITY_DISJUNCTIVE_QUERIES))
				{
					std::set<std::shared_ptr<ManagedReasoner>> reasonerChoices;
					for(auto &reasonerChoice : node2->reasonerChoices_)
						if(simple1->reasonerChoices_.find(reasonerChoice) != simple1->reasonerChoices_.end())
							reasonerChoices.insert(reasonerChoice);

					auto phi = createConnectiveFormula<DisjunctionFormula>(
							simple1->phi_, node2->phi_, FormulaType::DISJUNCTION);
					auto disjunctiveNode = std::make_shared<Node>(
							phi, reasonerChoices, PredicateType::FORMULA);
					initialNodes_.remove(simple1);
					initialNodes_.push_back(disjunctiveNode);
					simpleNodes.remove(simple1);
					simpleNodes.push_back(disjunctiveNode);
					hasNode = true;
					break;
				}
			}
			if(hasNode) continue;
		}
		initialNodes_.push_back(node2);
	}
}

void ReasoningGraph::conjunction(const ReasoningGraph &other)
{
	// create a copy of terminal nodes list as it is modified below
	auto terminals = terminalNodes();
	// look at each pair of terminal node of this, and initial node of the other graph.
	// if possible, create a conjunctive query node, else add edges between the
	// nodes to connect both graphs.
	for(auto &terminal1 : terminals)
	{
		for(auto &initial2 : other.initialNodes())
		{
			if(terminal1->canBeCombinedWith(initial2, CAPABILITY_CONJUNCTIVE_QUERIES)) {
				// create a conjunctive node (terminal1 AND initial2)
				addConjunctiveNode(terminal1, initial2);
			}
			else if(isEdgeNeeded(terminal1, initial2)) {
				// terminal1 and initial2 cannot be combined via conjunction.
				// but it could be that one of the nodes refers to a builtin
				// in which case the edge is omittable for some cases.
				addSuccessor(terminal1, initial2);
			}
		}
		// remove dangling node in this graph, i.e. terminal nodes not connected
		// to the other graph.
		// NOTE: dangling nodes in the other graph are not referred to in this graph, so can be ignored
		if(terminal1->successors_.empty()) removeNode(terminal1);
	}
}

void ReasoningGraph::addConjunctiveNode(const NodePtr &a, const NodePtr &b)
{
	std::set<std::shared_ptr<ManagedReasoner>> reasonerChoices;
	for(auto &r1 : a->reasonerChoices_)
		if(b->reasonerChoices_.find(r1) != b->reasonerChoices_.end()) reasonerChoices.insert(r1);
	// create a conjunctive node
	auto phi = createConnectiveFormula<ConjunctionFormula>(
			a->phi_, b->phi_, FormulaType::CONJUNCTION);
	auto conjunctiveNode = std::make_shared<Node>(
			phi, reasonerChoices, PredicateType::FORMULA);

	// connect the node to the rest of the graph
	if(a->predecessors_.empty()) {
		initialNodes_.push_back(conjunctiveNode);
	} else {
		for(auto &x : a->predecessors_)
			addSuccessor(x,conjunctiveNode);
	}
	if(b->successors_.empty()) {
		terminalNodes_.push_back(conjunctiveNode);
	} else {
		for(auto &x : b->successors_)
			addSuccessor(conjunctiveNode, x);
	}
}

bool ReasoningGraph::isEdgeNeeded(const NodePtr &a, const NodePtr &b)
{
	if(a->predicateType_ == PredicateType::BUILT_IN) {
		// no edge needed if phi_a is a builtin supported by reasoner_b
		if(b->isBuiltinSupported(*(PredicateFormula*)a->phi_.get()))
				return false;
	}
	if(b->predicateType_ == PredicateType::BUILT_IN) {
		// no edge needed if phi_b is a builtin supported by reasoner_a
		if(a->isBuiltinSupported(*(PredicateFormula*)b->phi_.get()))
			return false;
	}
	return true;
}


Blackboard::Blackboard(
		const std::shared_ptr<ReasonerManager> &reasonerManager,
		const std::shared_ptr<QueryResultQueue> &outputQueue,
		const std::shared_ptr<const Query> &goal)
		: reasonerManager_(reasonerManager),
		  builtinEvaluator_(std::make_shared<ManagedReasoner>("builtins", BuiltinEvaluator::get())),
		  outputQueue_(outputQueue),
		  goal_(goal)
{
	// decompose the reasoning task
	inputStream_ = std::make_shared<QueryResultBroadcaster>();
	inputChannel_ = QueryResultStream::Channel::create(inputStream_);
	outBroadcaster_ = std::make_shared<QueryResultBroadcaster>();

	// create a channel of outputQueue_, and add it as subscriber
	auto queueChannel = QueryResultStream::Channel::create(outputQueue_);
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
	inputChannel_->push(QueryResultStream::bos());
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
			auto args = ((ConjunctionFormula*)phi.get())->formulae();
			auto graph(decomposeFormula(args[0]));
			for(int i=1; i<args.size(); ++i) {
				graph.conjunction(decomposeFormula(args[i]));
			}
			return graph;
		}
		case FormulaType::DISJUNCTION: {
			auto args = ((DisjunctionFormula*)phi.get())->formulae();
			auto graph(decomposeFormula(args[0]));
			for(int i=1; i<args.size(); ++i) {
				graph.disjunction(decomposeFormula(args[i]));
			}
			return graph;
		}
		case FormulaType::PREDICATE: {
			return decomposePredicate(std::static_pointer_cast<PredicateFormula>(phi));
		}
		default:
			KB_WARN("Ignoring unknown formula type '{}'.", (int)phi->type());
			return {};
	}
}

ReasoningGraph Blackboard::decomposePredicate(const std::shared_ptr<PredicateFormula> &phi) const
{
	// get ensemble of reasoner
	auto predicateDescription = reasonerManager_->getPredicateDefinition(
			phi->predicate()->indicator());
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

void Blackboard::createReasoningPipeline(const std::shared_ptr<QueryResultBroadcaster> &inputStream,
										 const std::shared_ptr<QueryResultBroadcaster> &outputStream)
{
	auto graph = decomposeFormula(goal_->formula());
	for(auto &n0 : graph.initialNodes())
		createReasoningPipeline(inputStream, outputStream, n0);
}

void Blackboard::createReasoningPipeline( //NOLINT
		const std::shared_ptr<QueryResultBroadcaster> &pipelineInput,
		const std::shared_ptr<QueryResultBroadcaster> &pipelineOutput,
		const std::shared_ptr<ReasoningGraph::Node> &n0)
{
	// create a subquery for the predicate p
	std::shared_ptr<Query> subQuery = std::make_shared<Query>(n0->phi());
	// create an output stream for this node
	std::shared_ptr<QueryResultBroadcaster> nodeOutput = (n0->successors().empty() ?
			pipelineOutput : std::make_shared<QueryResultBroadcaster>());

	auto n0_reasoner = *n0->reasonerChoices().begin();
	if(n0->predicateType() == PredicateType::BUILT_IN) {
		// prefer to use BuiltinEvaluator
		auto p = ((PredicateFormula*)n0->phi().get())->predicate();
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

void Blackboard::createReasoningStep(const std::shared_ptr<ManagedReasoner> &r,
									 const std::shared_ptr<Query> &subQuery,
									 const std::shared_ptr<QueryResultBroadcaster> &stepInput,
									 const std::shared_ptr<QueryResultBroadcaster> &stepOutput)
{
	std::shared_ptr<Blackboard::Stream> reasonerIn;
	std::shared_ptr<QueryResultStream::Channel> reasonerInChan, reasonerOutChan;
	// create IO channels
	reasonerOutChan = QueryResultStream::Channel::create(stepOutput);
	reasonerIn = std::make_shared<Blackboard::Stream>(
			r,reasonerOutChan, subQuery);
	reasonerInChan = QueryResultStream::Channel::create(reasonerIn);
	stepInput->addSubscriber(reasonerInChan);
	// create a new segment
	reasonerInputs_.push_back(reasonerIn);
}


Blackboard::Stream::Stream(
	const std::shared_ptr<ManagedReasoner> &reasoner,
	const std::shared_ptr<QueryResultStream::Channel> &outputStream,
	const std::shared_ptr<Query> &goal)
: QueryResultStream(),
  queryID_(reinterpret_cast<std::uintptr_t>(this)),
  reasoner_(reasoner),
  outputStream_(outputStream),
  goal_(goal),
  isQueryOpened_(true),
  hasStopRequest_(false)
{
	// tell the reasoner that a new request has been made
	(*reasoner_)()->startQuery(queryID_, goal);
}

Blackboard::Stream::~Stream()
{
	stop();
}

void Blackboard::Stream::push(const QueryResultPtr &msg)
{
	if(QueryResultStream::isEOS(msg)) {
		// tell the reasoner to finish up.
		// if hasStopRequest_=true it means that the reasoner is requested
		// to immediately shutdown. however, note that not all reasoner
		// implementations may support this and may take longer to stop anyway.
		if(isQueryOpened()) {
			isQueryOpened_ = false;
			// FIXME: need to call finishQuery if exception happens in runner!!
			//          else consumer will never be called with EOS
			(*reasoner_)()->finishQuery(queryID_, outputStream_, hasStopRequest());
		}
	}
	else if(!isQueryOpened()) {
		KB_WARN("ignoring attempt to write to a closed stream.");
	}
	else {
		// evaluate an instance of the input query
		auto queryInstance = std::make_shared<QueryInstance>(
				goal_, outputStream_, msg);
		(*reasoner_)()->runQueryInstance(queryID_, queryInstance);
	}
}

void Blackboard::Stream::stop()
{
	hasStopRequest_ = true;
	close();
}
