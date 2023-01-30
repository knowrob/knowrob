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

	decomposeFormula(goal->formula(), NodeList());
	combineAdjacentNodes();
	createPipeline(inputStream_, outBroadcaster_);
}

Blackboard::~Blackboard()
{
	stop();
}

void Blackboard::print(const NodePtr &node) //NOLINT
{
	if(node) {
		if(node->successors_.empty()) {
			std::string reasonerList;
			for(auto &r : node->reasoner_) reasonerList += ":"+r->name();
			KB_INFO("terminal: {} {}", *node->phi_, reasonerList);
		}
		for(auto &successor : node->successors_) {
			KB_INFO("edge: {} -- {}", *node->phi_, *successor->phi_);
			print(successor);
		}
	} else {
		for(auto &n0 : initialNodes_) {
			print(n0);
		}
	}
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

void Blackboard::addSuccessor(const NodePtr &predecessor, const NodePtr &successor)
{
	predecessor->successors_.push_back(successor);
	successor->predecessors_.push_back(predecessor);
}

void Blackboard::removeSuccessor(const NodePtr &predecessor, const NodePtr &successor)
{
	predecessor->successors_.remove(successor);
	successor->predecessors_.remove(predecessor);
}

Blackboard::NodeList Blackboard::decomposeFormula( //NOLINT
		const std::shared_ptr<Formula> &phi, const NodeList &predecessors)
{
	switch(phi->type()) {
		case FormulaType::CONJUNCTION: {
			NodeList terminalNodes = predecessors;
			for(const std::shared_ptr<Formula> &psi : ((ConjunctionFormula*)phi.get())->formulae()) {
				terminalNodes = decomposeFormula(psi, terminalNodes);
			}
			return terminalNodes;
		}
		case FormulaType::DISJUNCTION: {
			NodeList terminalNodes;
			for(const std::shared_ptr<Formula> &psi : ((DisjunctionFormula*)phi.get())->formulae()) {
				terminalNodes.splice(terminalNodes.end(), decomposeFormula(psi, predecessors));
			}
			return terminalNodes;
		}
		case FormulaType::PREDICATE: {
			return decomposePredicate(std::static_pointer_cast<PredicateFormula>(phi), predecessors);
		}
		default:
			KB_WARN("Ignoring unknown formula type '{}'.", (int)phi->type());
			return {};
	}
}

Blackboard::NodeList Blackboard::decomposePredicate(
		const std::shared_ptr<PredicateFormula> &phi, const NodeList &predecessors)
{
	// get ensemble of reasoner
	auto predicateDescription = reasonerManager_->getPredicateDefinition(
			phi->predicate()->indicator());
	if (!predicateDescription || predicateDescription->reasonerEnsemble().empty()) {
		throw QueryError("no reasoner registered for query `{}`", *phi);
	}

	NodeList out; {
		auto newNode = std::make_shared<Node>(phi,
				predicateDescription->reasonerEnsemble(),
				predicateDescription->predicateType());
		if(predecessors.empty()) {
			initialNodes_.push_back(newNode);
		}
		else {
			for(auto &n : predecessors) {
				addSuccessor(n,newNode);
			}
		}
		out.push_back(newNode);
	}
	return out;
}

void Blackboard::combineAdjacentNodes()
{
	// note: combineAdjacentNodes currently may generate redundant evaluation graph nodes
	//  - it generates a kind of canonical form e.g. a query
	//  		`p1,((p2,p3);(p4,(p5;p6)))` may generate a graph representing the query
	//  		`(p1,p2,p3);(p1,p4,p5);(p1,p4,p6)` with redundant goals.
	//	- TODO: start with the innermost term, and combine generation of disjunctions and conjunctions
	//           but current graph structure doesn't support this nicely
	// - FIXME: might be some redundant nodes are generated for builtins
	//
	for(auto it = initialNodes_.begin(); it != initialNodes_.end();)
		createConjunctiveQueries(*(it++));
	createDisjunctiveQueries(initialNodes_);
}

void Blackboard::createConjunctiveQueries( //NOLINT
		const std::shared_ptr<Node> &node)
{
	bool hasSuccessors = !node->successors_.empty();
	for(auto it = node->successors_.begin(); it !=  node->successors_.end();) {
		auto successor = *(it++);
		std::set<std::shared_ptr<ManagedReasoner>> combinableReasoner;
		// find list of reasoner shared between both nodes that are capable
		// of evaluating conjunctive queries.
		std::copy_if(node->reasoner_.begin(), node->reasoner_.end(),
					 std::inserter(combinableReasoner, combinableReasoner.begin()),
					 [successor](auto &r){
			return r->reasoner()->hasCapability(CAPABILITY_CONJUNCTIVE_QUERIES) &&
					successor->reasoner_.find(r)!=successor->reasoner_.end();
		});
		if(combinableReasoner.empty()) {
			createConjunctiveQueries(successor);
		}
		else {
			// conjunctive query is possible, remove old successor relation
			removeSuccessor(node, successor);

			// create node for the conjunctive query
			auto conjunction = createConnectiveFormula<ConjunctionFormula>(
					node->phi_, successor->phi_, FormulaType::CONJUNCTION);
			auto conjunctiveNode = std::make_shared<Node>(
					conjunction, combinableReasoner, PredicateType::FORMULA);
			if(node->predecessors_.empty()) {
				initialNodes_.push_front(conjunctiveNode);
			} else {
				for(auto &x : node->predecessors_)
					addSuccessor(x, conjunctiveNode);
			}
			for(auto &x : successor->successors_)
				addSuccessor(conjunctiveNode, x);
			createConjunctiveQueries(conjunctiveNode);

			// handle case that not all reasoner of node can be combined with the successor
			if(combinableReasoner.size() < node->reasoner_.size()) {
				// get set of reasoner without combinableReasoner
				std::set<std::shared_ptr<ManagedReasoner>> remainderReasoner;
				std::copy_if(node->reasoner_.begin(), node->reasoner_.end(),
							 std::inserter(remainderReasoner, remainderReasoner.begin()),
							 [combinableReasoner](auto &r){
								 return combinableReasoner.find(r)==combinableReasoner.end();
							 });
				// create remainder node
				auto remainderNode = std::make_shared<Node>(
						node->phi_, remainderReasoner, node->predicateType_);
				if(node->predecessors_.empty()) {
					initialNodes_.push_front(remainderNode);
				} else {
					for(auto &x : node->predecessors_)
						addSuccessor(x, remainderNode);
				}
				addSuccessor(remainderNode, successor);
				createConjunctiveQueries(successor);
			}
		}
	}
	if(hasSuccessors && node->successors_.empty()) {
		// all successors have been merged into a conjunctive query above
		if(node->predecessors_.empty()) {
			// node was an initial node
			initialNodes_.remove(node);
		}
		else {
			// also remove node as successor for all of its predecessors
			// note: dangling node is deleted automatically thanks to smart pointer.
			for(auto it = node->predecessors_.begin(); it != node->predecessors_.end();) {
				removeSuccessor(*(it++), node);
			}
		}
	}
}

void Blackboard::createDisjunctiveQueries(NodeList &successors)
{
	if(successors.size()>1) {
		// iterate over all alternatives, and find other alternatives that use the same reasoner
		for(auto it1 = successors.begin(); it1 !=  successors.end();) {
			auto &node = *(it1++);

			// go through each reasoner of node
			for(auto jt1 = node->reasoner_.begin(); jt1 != node->reasoner_.end();) {
				auto successorReasoner = *(jt1++);
				auto disjunction = node->phi_;
				std::set<NodePtr> disjunctionPredecessors, disjunctionSuccessors;

				// find other alternatives that share the reasoner
				for(auto it2 = it1; it2 !=  successors.end();) {
					auto &otherNode = *(it2++);
					auto needle = otherNode->reasoner_.find(successorReasoner);
					if(needle != otherNode->reasoner_.end()) {
						// otherNode can be merged with node
						// remove reasoner from otherNode
						otherNode->reasoner_.erase(needle);
						// add argument to disjunction term
						disjunction = createConnectiveFormula<DisjunctionFormula>(
								disjunction, otherNode->phi_, FormulaType::DISJUNCTION);
						// remember predecessors and successors of otherSuccessor
						disjunctionPredecessors.insert(otherNode->predecessors_.begin(), otherNode->predecessors_.end());
						disjunctionSuccessors.insert(otherNode->successors_.begin(), otherNode->successors_.end());
					}
				}

				if(disjunction != node->phi_) {
					// some nodes were merged
					// reasoner can be removed from node
					node->reasoner_.erase(successorReasoner);
					// make sure node predecessors and successors are included
					disjunctionPredecessors.insert(node->predecessors_.begin(), node->predecessors_.end());
					disjunctionSuccessors.insert(node->successors_.begin(), node->successors_.end());

					auto disjunctiveNode = std::make_shared<Node>(
							disjunction, successorReasoner, PredicateType::FORMULA);
					if(disjunctionPredecessors.empty()) {
						initialNodes_.push_front(disjunctiveNode);
					} else {
						for(auto &x : disjunctionPredecessors) {
							x->successors_.push_front(disjunctiveNode);
							disjunctiveNode->predecessors_.push_back(x);
						}
					}
					for(auto &x : disjunctionSuccessors)
						addSuccessor(disjunctiveNode, x);
				}
			}
		}
	}

	// remove all option without reasoner assigned
	for(auto it1 = successors.begin(); it1 !=  successors.end();) {
		auto &successor = *(it1++);
		if(successor->reasoner_.empty()) {
			successors.remove(successor);
		}
	}

	// TODO: proceed for successors recursively
}

template<class T> std::shared_ptr<T> Blackboard::createConnectiveFormula(
		const FormulaPtr &phi1, const FormulaPtr &phi2, FormulaType type)
{
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

void Blackboard::createPipeline(const std::shared_ptr<QueryResultBroadcaster> &inputStream,
								const std::shared_ptr<QueryResultBroadcaster> &outputStream)
{
	for(auto &n0 : initialNodes_)
		createPipeline(inputStream, outputStream, n0);
}

void Blackboard::createPipeline( //NOLINT
		const std::shared_ptr<QueryResultBroadcaster> &pipelineInput,
		const std::shared_ptr<QueryResultBroadcaster> &pipelineOutput,
		const std::shared_ptr<Node> &n0)
{
	// create a subquery for the predicate p
	std::shared_ptr<Query> subQuery = std::make_shared<Query>(n0->phi_);
	// create an output stream for this node
	std::shared_ptr<QueryResultBroadcaster> nodeOutput = (n0->successors_.empty() ?
			pipelineOutput : std::make_shared<QueryResultBroadcaster>());

	if(n0->predicateType_ == PredicateType::BUILT_IN) {
		auto builtInReasoner = selectBuiltInReasoner(n0->phi_, n0->reasoner_);
		createReasonerPipeline(builtInReasoner, subQuery, pipelineInput, nodeOutput);
	}
	else {
		// add a node in communication graph for each reasoner in list
		for(auto &r : n0->reasoner_)
			createReasonerPipeline(r, subQuery, pipelineInput, nodeOutput);
	}

	// continue for successors
	for(auto &successor : n0->successors_) {
		createPipeline(nodeOutput, pipelineOutput, successor);
	}
}

std::shared_ptr<ManagedReasoner> Blackboard::selectBuiltInReasoner(
		const FormulaPtr &phi,
		const std::set<std::shared_ptr<ManagedReasoner>> &setOfReasoner)
{
	// prefer to use BuiltinEvaluator
	if(phi->type() == FormulaType::PREDICATE) {
		auto p = ((PredicateFormula*)phi.get())->predicate();
		if(BuiltinEvaluator::get()->isBuiltinSupported(p->indicator())) {
			return builtinEvaluator_;
		}
	}
	// else pick any from setOfReasoner
	if(setOfReasoner.empty()) {
		return {};
	}
	else {
		return *setOfReasoner.begin();
	}
}

void Blackboard::createReasonerPipeline(const std::shared_ptr<ManagedReasoner> &r,
										const std::shared_ptr<Query> &subQuery,
										const std::shared_ptr<QueryResultBroadcaster> &pipelineInput,
										const std::shared_ptr<QueryResultBroadcaster> &nodeOutput)
{
	std::shared_ptr<Blackboard::Stream> reasonerIn;
	std::shared_ptr<QueryResultStream::Channel> reasonerInChan, reasonerOutChan;
	// create IO channels
	reasonerOutChan = QueryResultStream::Channel::create(nodeOutput);
	reasonerIn = std::make_shared<Blackboard::Stream>(
			r,reasonerOutChan, subQuery);
	reasonerInChan = QueryResultStream::Channel::create(reasonerIn);
	pipelineInput->addSubscriber(reasonerInChan);
	// create a new segment
	reasonerInputs_.push_back(reasonerIn);
}


Blackboard::Node::Node(const std::shared_ptr<Formula> &phi,
					   const std::set<std::shared_ptr<ManagedReasoner>> &reasoner,
					   PredicateType predicateType)
		: phi_(phi), reasoner_(reasoner), predicateType_(predicateType)
{
}

Blackboard::Node::Node(const std::shared_ptr<Formula> &phi,
					   const std::shared_ptr<ManagedReasoner> &reasoner,
					   PredicateType predicateType)
		: phi_(phi), predicateType_(predicateType)
{
	reasoner_.insert(reasoner);
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
