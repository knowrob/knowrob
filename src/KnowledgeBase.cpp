/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <thread>
#include <utility>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <knowrob/Logger.h>
#include <knowrob/KnowledgeBase.h>
#include <knowrob/URI.h>
#include "knowrob/semweb/PrefixRegistry.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/queries/QueryTree.h"
#include "knowrob/queries/ConjunctiveBroadcaster.h"
#include "knowrob/queries/IDBStage.h"
#include "knowrob/queries/EDBStage.h"
#include "knowrob/queries/NegationStage.h"
#include "knowrob/queries/ModalStage.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/KnowledgeBaseError.h"
#include "knowrob/queries/RedundantAnswerFilter.h"
#include "knowrob/queries/DisjunctiveBroadcaster.h"
#include "knowrob/queries/AnswerYes.h"

using namespace knowrob;

namespace knowrob {
	struct EDBComparator {
		inline bool operator()(const RDFLiteralPtr &l1, const RDFLiteralPtr &l2) {
			// note: using ">" in return statements means that smaller element appears before larger.
			// note: this heuristic ignores dependencies.

			// (1) prefer evaluation of literals with fewer variables
			auto numVars1 = l1->numVariables();
			auto numVars2 = l2->numVariables();
			if (numVars1 < numVars2) return (numVars1 > numVars2);

			// TODO: (2) prefer evaluation of literals with less EDB assertions

			return (l1 < l2);
		}
	};

	// used to sort nodes in a priority queue.
	// the priority value is used to determine which nodes should be evaluated first.
	struct CompareNodes {
		bool operator()(const DependencyNodePtr &a, const DependencyNodePtr &b) const {
			// note: using ">" in return statements means that smaller element appears before larger.
			if (a->numVariables() != b->numVariables()) {
				// prefer node with less variables
				return a->numVariables() > b->numVariables();
			}
			if (a->numNeighbors() != b->numNeighbors()) {
				// prefer node with less neighbors
				return a->numNeighbors() > b->numNeighbors();
			}
			return a < b;
		}
	};

	// represents a possible step in the query pipeline
	struct QueryPipelineNode {
		explicit QueryPipelineNode(const DependencyNodePtr &node)
				: node_(node) {
			// add all nodes to a priority queue
			for (auto &neighbor: node->neighbors()) {
				neighbors_.push(neighbor);
			}
		}

		const DependencyNodePtr node_;
		std::priority_queue<DependencyNodePtr, std::vector<DependencyNodePtr>, CompareNodes> neighbors_;
	};

	using QueryPipelineNodePtr = std::shared_ptr<QueryPipelineNode>;

	class AnswerBuffer_WithReference : public TokenBuffer {
	public:
		explicit AnswerBuffer_WithReference(const std::shared_ptr<QueryPipeline> &pipeline)
				: TokenBuffer(), pipeline_(pipeline) {}

	protected:
		std::shared_ptr<QueryPipeline> pipeline_;
	};
}

KnowledgeBase::KnowledgeBase(const boost::property_tree::ptree &config)
		: threadPool_(std::make_shared<ThreadPool>(std::thread::hardware_concurrency())) {
	backendManager_ = std::make_shared<BackendManager>(threadPool_);
	reasonerManager_ = std::make_shared<ReasonerManager>(threadPool_, backendManager_);
	loadConfiguration(config);
}

KnowledgeBase::KnowledgeBase(const std::string_view &configFile)
		: threadPool_(std::make_shared<ThreadPool>(std::thread::hardware_concurrency())) {
	backendManager_ = std::make_shared<BackendManager>(threadPool_);
	reasonerManager_ = std::make_shared<ReasonerManager>(threadPool_, backendManager_);

	boost::property_tree::ptree config;
	boost::property_tree::read_json(URI::resolve(configFile), config);
	loadConfiguration(config);
}

void KnowledgeBase::loadConfiguration(const boost::property_tree::ptree &config) {
	auto semwebTree = config.get_child_optional("semantic-web");
	if (semwebTree) {
		// load RDF URI aliases
		auto prefixesList = semwebTree.value().get_child_optional("prefixes");
		for (const auto &pair: prefixesList.value()) {
			auto alias = pair.second.get("alias", "");
			auto uri = pair.second.get("uri", "");
			if (!alias.empty() && !uri.empty()) {
				semweb::PrefixRegistry::get().registerPrefix(alias, uri);
			} else {
				KB_WARN("Invalid entry in semantic-web::prefixes, 'alias' and 'uri' must be defined.");
			}
		}
	}

	// initialize RDF data backends from configuration
	auto backendList = config.get_child_optional("data-backends");
	if (backendList) {
		for (const auto &pair: backendList.value()) {
			try {
				backendManager_->loadBackend(pair.second);
			}
			catch (std::exception &e) {
				KB_ERROR("failed to load a knowledgeGraph: {}", e.what());
			}
		}
	} else {
		KB_ERROR("configuration has no 'backends' key.");
	}

	// get the central backend, ensure it implements KnowledgeGraph class.
	auto centralBackendName = config.get_optional<std::string>("central-backend");
	if (centralBackendName) {
		auto centralBackend = backendManager_->getBackendWithID(centralBackendName.value());
		if (!centralBackend) {
			KB_ERROR("failed to find central backend with ID '{}'.", centralBackendName.value());
		}
		centralKG_ = std::dynamic_pointer_cast<KnowledgeGraph>(centralBackend->backend());
		if (!centralKG_) {
			KB_ERROR("backend '{}' is not a type of KnowledgeGraph.", centralBackendName.value());
		}
	} else {
		KB_ERROR("configuration has no 'central-backend' key.");
	}
	if (!centralKG_) {
		throw KnowledgeBaseError("failed to initialize central knowledge graph.");
	}

	// load reasoners from configuration
	auto reasonerList = config.get_child_optional("reasoner");
	if (reasonerList) {
		for (const auto &pair: reasonerList.value()) {
			try {
				auto definedReasoner = reasonerManager_->loadReasoner(pair.second);
				// if reasoner implements DataBackend class, add it to the backend manager
				auto reasonerBackend = std::dynamic_pointer_cast<DataBackend>(definedReasoner->reasoner());
				if (reasonerBackend) {
					auto backendID = definedReasoner->name() + "__BACKEND";
					backendManager_->addBackend(backendID, reasonerBackend);
				}
			}
			catch (std::exception &e) {
				KB_ERROR("failed to load a reasoner: {}", e.what());
			}
		}
	} else {
		KB_ERROR("configuration has no 'reasoner' key.");
	}

	// load the "global" data sources.
	// these are data sources that are loaded into all backends, however
	// the backends may decide to ignore some of the data sources.
	auto dataSourcesList = config.get_child_optional("data-sources");
	if (dataSourcesList) {
		static const std::string formatDefault = {};

		for (const auto &pair: dataSourcesList.value()) {
			auto &subtree = pair.second;
			auto dataFormat = subtree.get("format", formatDefault);
			auto source = std::make_shared<DataSource>(dataFormat);
			source->loadSettings(subtree);

			for (auto &kg_pair: backendManager_->backendPool()) {
				kg_pair.second->backend()->loadDataSource(source);
			}
		}
	}
}

bool KnowledgeBase::isMaterializedInEDB(std::string_view property) const {
	return centralKG()->isDefinedProperty(property);
}

std::vector<RDFComputablePtr> KnowledgeBase::createComputationSequence(
		const std::list<DependencyNodePtr> &dependencyGroup) {
	// Pick a node to start with.
	// For now the one with minimum number of neighbors is picked.
	DependencyNodePtr first;
	unsigned long minNumNeighbors = 0;
	for (auto &n: dependencyGroup) {
		if (!first || n->numNeighbors() < minNumNeighbors) {
			minNumNeighbors = n->numNeighbors();
			first = n;
		}
		else if(minNumNeighbors == n->numNeighbors() && n->numVariables() < first->numVariables()) {
			first = n;
		}
	}

	// remember visited nodes, needed for circular dependencies
	// all nodes added to the queue should also be added to this set.
	std::set<DependencyNode *> visited;
	visited.insert(first.get());

	std::vector<RDFComputablePtr> sequence;
	sequence.push_back(std::static_pointer_cast<RDFComputable>(first->literal()));

	// start with a FIFO queue only containing first node
	std::deque<QueryPipelineNodePtr> queue;
	auto qn0 = std::make_shared<QueryPipelineNode>(first);
	queue.push_front(qn0);

	// loop until queue is empty and process exactly one successor of
	// the top element in the FIFO in each step. If an element has no
	// more successors, it can be removed from queue.
	// Each successor creates an additional node added to the top of the FIFO.
	while (!queue.empty()) {
		auto front = queue.front();

		// get top successor node that has not been visited yet
		DependencyNodePtr topNext;
		while (!front->neighbors_.empty()) {
			auto topNeighbor = front->neighbors_.top();
			front->neighbors_.pop();

			if (visited.count(topNeighbor.get()) == 0) {
				topNext = topNeighbor;
				break;
			}
		}
		// pop element from queue if all neighbors were processed
		if (front->neighbors_.empty()) {
			queue.pop_front();
		}

		if (topNext) {
			// push a new node onto FIFO
			auto qn_next = std::make_shared<QueryPipelineNode>(topNext);
			queue.push_front(qn_next);
			sequence.push_back(std::static_pointer_cast<RDFComputable>(topNext->literal()));
			visited.insert(topNext.get());
		}
	}

	return sequence;
}

void KnowledgeBase::createComputationPipeline(
		const std::shared_ptr<QueryPipeline> &pipeline,
		const std::vector<RDFComputablePtr> &computableLiterals,
		const std::shared_ptr<TokenBroadcaster> &pipelineInput,
		const std::shared_ptr<TokenBroadcaster> &pipelineOutput,
		const QueryContextPtr &ctx) {
	// This function generates a query pipeline for literals that can be computed
	// (EDB-only literals are processed separately). The literals are part of one dependency group.
	// They are sorted, and also evaluated in this order. For each computable literal there is at
	// least one reasoner that can compute the literal. However, instances of the literal may also
	// occur in the EDB. Hence, computation results must be combined with results of an EDB query
	// for each literal.

	auto lastOut = pipelineInput;
	auto edb = centralKG();

	for (auto &lit: computableLiterals) {
		auto stepInput = lastOut;
		auto stepOutput = std::make_shared<TokenBroadcaster>();
		pipeline->addStage(stepOutput);

		// --------------------------------------
		// Construct a pipeline that grounds the literal in the EDB.
		// But only add an EDB stage if the predicate was materialized in EDB before,
		// or if the predicate is a variable.
		// --------------------------------------
		bool isEDBStageNeeded = true;
		if (lit->propertyTerm() && lit->propertyTerm()->type() == TermType::STRING) {
			auto st = (StringTerm *) lit->propertyTerm().get();
			isEDBStageNeeded = isMaterializedInEDB(st->value());
		}
		if (isEDBStageNeeded) {
			auto edbStage = std::make_shared<EDBStage>(edb, lit, ctx);
			edbStage->selfWeakRef_ = edbStage;
			stepInput >> edbStage;
			edbStage >> stepOutput;
			pipeline->addStage(edbStage);
		}

		// --------------------------------------
		// Construct a pipeline that grounds the literal in the IDB.
		// To this end add an IDB stage for each reasoner that defines the literal.
		// --------------------------------------
		for (auto &r: lit->reasonerList()) {
			auto idbStage = std::make_shared<IDBStage>(r, lit, threadPool_, ctx);
			idbStage->selfWeakRef_ = idbStage;
			stepInput >> idbStage;
			idbStage >> stepOutput;
			pipeline->addStage(idbStage);
			// TODO: what about the materialization of the predicate in EDB?
			//       there is also the QUERY_FLAG_PERSIST_SOLUTIONS flag
		}

		// add a stage that consolidates the results of the EDB and IDB stages.
		// in particular the case needs to be handled where none of the stages return
		// 'true'. Also print a warning if two stages disagree but state they are confident.
		auto consolidator = std::make_shared<DisjunctiveBroadcaster>();
		stepOutput >> consolidator;
		// TODO: are all these addStage calls really needed?
		pipeline->addStage(consolidator);

		// --------------------------------------
		// Optionally add a stage to the pipeline that drops all redundant result.
		// The filter is applied here to remove redundancies early on directly after IDB and EDB
		// results are combined.
		// --------------------------------------
		if (ctx->queryFlags_ & QUERY_FLAG_UNIQUE_SOLUTIONS) {
			auto filterStage = std::make_shared<RedundantAnswerFilter>();
			stepOutput >> filterStage;
			lastOut = filterStage;
		} else {
			lastOut = stepOutput;
		}
	}

	lastOut >> pipelineOutput;
}

TokenBufferPtr KnowledgeBase::submitQuery(const ConjunctiveQueryPtr &graphQuery) {
	auto allLiterals = graphQuery->literals();

	// --------------------------------------
	// Construct a pipeline that holds references to stages.
	// --------------------------------------
	auto pipeline = std::make_shared<QueryPipeline>();

	// --------------------------------------
	// Pick a Knowledge Graph for EDB queries
	// --------------------------------------
	std::shared_ptr<KnowledgeGraph> kg = centralKG();

	// --------------------------------------
	// split input literals into positive and negative literals.
	// negative literals are evaluated in parallel after all positive literals.
	// --------------------------------------
	std::vector<RDFLiteralPtr> positiveLiterals, negativeLiterals;
	for (auto &l: allLiterals) {
		if (l->isNegated()) negativeLiterals.push_back(l);
		else positiveLiterals.push_back(l);
	}

	// --------------------------------------
	// sort positive literals. the EDB might evaluate literals in the given order.
	// --------------------------------------
	std::sort(positiveLiterals.begin(), positiveLiterals.end(), EDBComparator());

	// --------------------------------------
	// split positive literals into edb-only and computable.
	// also associate list of reasoner to computable literals.
	// --------------------------------------
	std::vector<RDFLiteralPtr> edbOnlyLiterals;
	std::vector<RDFComputablePtr> computableLiterals;
	for (auto &l: positiveLiterals) {
		std::vector<std::shared_ptr<Reasoner>> l_reasoner;
		for (auto &pair: reasonerManager_->reasonerPool()) {
			auto &r = pair.second->reasoner();
			auto descr = r->getLiteralDescription(*l);
			if (descr) {
				// TODO: check descr, e.g. about materialization in EDB
				l_reasoner.push_back(r);
			}
		}
		if (l_reasoner.empty()) {
			edbOnlyLiterals.push_back(l);
			auto l_p = l->propertyTerm();
			if(l_p && l_p->type()==TermType::STRING &&
			  !isMaterializedInEDB(std::static_pointer_cast<StringTerm>(l_p)->value())) {
			  	// generate a "don't know" message and return.
				auto out = std::make_shared<TokenBuffer>();
				auto channel = TokenStream::Channel::create(out);
				auto dontKnow = std::make_shared<AnswerDontKnow>();
				KB_WARN("Predicate is neither materialized in EDB nor defined by a reasoner: {}",
						*l->predicate());
				channel->push(dontKnow);
				channel->push(EndOfEvaluation::get());
				return out;
			}
		} else {
			computableLiterals.push_back(std::make_shared<RDFComputable>(*l, l_reasoner));
		}
	}

	// --------------------------------------
	// run EDB query with all edb-only literals.
	// --------------------------------------
	std::shared_ptr<TokenBuffer> edbOut;
	if (edbOnlyLiterals.empty()) {
		edbOut = std::make_shared<TokenBuffer>();
		auto channel = TokenStream::Channel::create(edbOut);
		channel->push(GenericYes());
		channel->push(EndOfEvaluation::get());
	} else {
		auto edbOnlyQuery = std::make_shared<ConjunctiveQuery>(
				edbOnlyLiterals, graphQuery->ctx());
		edbOut = kg->submitQuery(edbOnlyQuery);
	}
	pipeline->addStage(edbOut);

	// --------------------------------------
	// handle positive IDB literals.
	// --------------------------------------
	std::shared_ptr<TokenBroadcaster> idbOut;
	if (computableLiterals.empty()) {
		idbOut = edbOut;
	} else {
		idbOut = std::make_shared<TokenBroadcaster>();
		pipeline->addStage(idbOut);
		// --------------------------------------
		// Compute dependency groups of computable literals.
		// --------------------------------------
		DependencyGraph dg;
		dg.insert(computableLiterals.begin(), computableLiterals.end());

		// --------------------------------------
		// Construct a pipeline for each dependency group.
		// --------------------------------------
		if (dg.numGroups() == 1) {
			auto &literalGroup = *dg.begin();
			createComputationPipeline(
					pipeline,
					createComputationSequence(literalGroup.member_),
					edbOut,
					idbOut,
					graphQuery->ctx());
		} else {
			// there are multiple dependency groups. They can be evaluated in parallel.

			// combines sub-answers computed in different parallel steps
			auto answerCombiner = std::make_shared<ConjunctiveBroadcaster>();
			// create a parallel step for each dependency group
			for (auto &literalGroup: dg) {
				// --------------------------------------
				// Construct a pipeline for each dependency group.
				// --------------------------------------
				createComputationPipeline(
						pipeline,
						createComputationSequence(literalGroup.member_),
						edbOut,
						answerCombiner,
						graphQuery->ctx());
			}
			answerCombiner >> idbOut;
			pipeline->addStage(answerCombiner);
		}
	}

	// --------------------------------------
	// Evaluate all negative literals in parallel.
	// --------------------------------------
	std::shared_ptr<TokenBroadcaster> lastStage;
	if (!negativeLiterals.empty()) {
		// run a dedicated stage where negated literals can be evaluated in parallel
		auto negStage = std::make_shared<LiteralNegationStage>(
				this, graphQuery->ctx(), negativeLiterals);
		idbOut >> negStage;
		lastStage = negStage;
	} else {
		lastStage = idbOut;
	}

	auto out = std::make_shared<AnswerBuffer_WithReference>(pipeline);
	lastStage >> out;
	edbOut->stopBuffering();
	return out;
}

TokenBufferPtr KnowledgeBase::submitQuery(const LiteralPtr &literal, const QueryContextPtr &ctx) {
	auto rdfLiteral = std::make_shared<RDFLiteral>(
			literal->predicate(), literal->isNegated(), ctx->selector_);
	return submitQuery(std::make_shared<ConjunctiveQuery>(
			ConjunctiveQuery({rdfLiteral}, ctx)));
}

TokenBufferPtr KnowledgeBase::submitQuery(const FormulaPtr &phi, const QueryContextPtr &ctx) {
	auto outStream = std::make_shared<TokenBuffer>();

	auto pipeline = std::make_shared<QueryPipeline>();
	pipeline->addStage(outStream);

	// --------------------------------------
	// Pick a Knowledge Graph for EDB queries
	// --------------------------------------
	std::shared_ptr<KnowledgeGraph> kg = centralKG();

	// decompose input formula into parts that are considered in disjunction,
	// and thus can be evaluated in parallel.
	QueryTree qt(phi);
	for (auto &path: qt) {
		// each node in a path is either a predicate, a negated predicate,
		// a modal formula, or the negation of a modal formula.

		std::vector<RDFLiteralPtr> posLiterals, negLiterals;
		std::vector<std::shared_ptr<ModalFormula>> posModals, negModals;

		for (auto &node: path.nodes()) {
			switch (node->type()) {
				case FormulaType::PREDICATE:
					posLiterals.push_back(std::make_shared<RDFLiteral>(
							std::static_pointer_cast<Predicate>(node),
							false, ctx->selector_));
					break;

				case FormulaType::MODAL:
					posModals.push_back(std::static_pointer_cast<ModalFormula>(node));
					break;

				case FormulaType::NEGATION: {
					auto negation = (Negation *) node.get();
					auto negated = negation->negatedFormula();
					switch (negated->type()) {
						case FormulaType::PREDICATE:
							negLiterals.push_back(std::make_shared<RDFLiteral>(
									std::static_pointer_cast<Predicate>(negated),
									true, ctx->selector_));
							break;
						case FormulaType::MODAL:
							negModals.push_back(std::static_pointer_cast<ModalFormula>(negated));
							break;
						default:
							throw QueryError("Unexpected negated formula type {} in QueryTree.", (int) negated->type());
					}
					break;
				}
				default:
					throw QueryError("Unexpected formula type {} in QueryTree.", (int) node->type());
			}
		}

		std::shared_ptr<TokenBroadcaster> lastStage;
		std::shared_ptr<TokenBuffer> firstBuffer;

		// first evaluate positive literals if any
		// note that the first stage is buffered, so that the next stage can be added to the pipeline
		// and only after stopping the buffering messages will be forwarded to the next stage.
		if (posLiterals.empty()) {
			// if there are none, we still need to indicate begin and end of stream for the rest of the pipeline.
			// so we just push `bos` (an empty substitution) followed by `eos` and feed these messages to the next stage.
			firstBuffer = std::make_shared<TokenBuffer>();
			lastStage = firstBuffer;
			auto channel = TokenStream::Channel::create(lastStage);
			channel->push(GenericYes());
			channel->push(EndOfEvaluation::get());
			pipeline->addStage(lastStage);
		} else {
			auto pathQuery = std::make_shared<ConjunctiveQuery>(posLiterals, ctx);
			firstBuffer = submitQuery(pathQuery);
			lastStage = firstBuffer;
			pipeline->addStage(lastStage);
		}

		// --------------------------------------
		// Evaluate all positive modals in sequence.
		// TODO: compute dependency between modals, evaluate independent modals in parallel.
		//       they could also be independent in evaluation context only, we could check which variables receive
		//       grounding already before in posLiteral query!
		//       this is effectively what is done in submitQuery(pathQuery)
		// --------------------------------------
		for (auto &posModal: posModals) {
			// FIXME: use of this pointer below. only ok if destructor makes sure to destroy all pipelines.
			//        maybe a better way would be having another class generating pipelines with the ability
			//        to create reference pointer on a KnowledgeBase
			auto modalStage = std::make_shared<ModalStage>(this, posModal, ctx);
			modalStage->selfWeakRef_ = modalStage;
			lastStage >> modalStage;
			lastStage = modalStage;
			pipeline->addStage(lastStage);
		}

		// --------------------------------------
		// Evaluate all negative literals in parallel.
		// --------------------------------------
		if (!negLiterals.empty()) {
			// run a dedicated stage where negated literals can be evaluated in parallel
			auto negLiteralStage = std::make_shared<LiteralNegationStage>(
					this, ctx, negLiterals);
			lastStage >> negLiteralStage;
			lastStage = negLiteralStage;
			pipeline->addStage(lastStage);
		}

		// --------------------------------------
		// Evaluate all negative modals in parallel.
		// --------------------------------------
		if (!negModals.empty()) {
			// run a dedicated stage where negated modals can be evaluated in parallel
			auto negModalStage = std::make_shared<ModalNegationStage>(
					this, ctx, negModals);
			lastStage >> negModalStage;
			lastStage = negModalStage;
			pipeline->addStage(lastStage);
		}

		lastStage >> outStream;
		firstBuffer->stopBuffering();
		pipeline->addStage(lastStage);
	}
	// At this point outStream could already contain solutions, but these are buffered
	// such that they won't be lost during pipeline creation.

	// if there were multiple paths, consolidate answers from them.
	// e.g. if one yields no and the other true, the no should be ignored.
	std::shared_ptr<TokenBroadcaster> finalStage;
	if (qt.numPaths() > 1) {
		auto consolidator = std::make_shared<DisjunctiveBroadcaster>();
		outStream >> consolidator;
		finalStage = consolidator;
	} else {
		finalStage = outStream;
	}

	// Finally, wrap output into AnswerBuffer_WithReference object.
	// Note that the AnswerBuffer_WithReference object is used such that the caller can
	// destroy the whole pipeline by de-referencing the returned AnswerBufferPtr.
	auto out = std::make_shared<AnswerBuffer_WithReference>(pipeline);
	finalStage >> out;
	outStream->stopBuffering();
	return out;
}

bool KnowledgeBase::insert(const std::vector<StatementData> &propositions) {
	bool status = true;
	for (auto &kg: backendManager_->backendPool()) {
		if (!kg.second->backend()->insertAll(propositions)) {
			KB_WARN("assertion of triple data failed!");
			status = false;
		}
	}
	return status;
}

bool KnowledgeBase::insert(const StatementData &proposition) {
	bool status = true;
	// assert each statement into each knowledge graph backend
	for (auto &kg: backendManager_->backendPool()) {
		if (!kg.second->backend()->insertOne(proposition)) {
			KB_WARN("assertion of triple data failed!");
			status = false;
		}
	}
	return status;
}
