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
#include "knowrob/queries/AnswerCombiner.h"
#include "knowrob/queries/IDBStage.h"
#include "knowrob/queries/EDBStage.h"
#include "knowrob/queries/NegationStage.h"
#include "knowrob/queries/ModalStage.h"
#include "knowrob/queries/QueryError.h"

using namespace knowrob;

namespace knowrob
{
    struct EDBComparator
    {
        inline bool operator() (const RDFLiteralPtr& l1, const RDFLiteralPtr& l2)
        {
            // note: using ">" in return statements means that smaller element appears before larger.
            // note: this heuristic ignores dependencies.

            // (1) prefer evaluation of literals with fewer variables
            auto numVars1 = l1->numVariables();
            auto numVars2 = l2->numVariables();
            if(numVars1 < numVars2) return (numVars1 > numVars2);

            // TODO: (2) prefer evaluation of literals with less EDB assertions

            return (l1 < l2);
        }
    };

    // used to sort nodes in a priority queue.
    // the priority value is used to determine which nodes should be evaluated first.
    struct CompareNodes
    {
        bool operator()(const DependencyNodePtr &a, const DependencyNodePtr &b) const
        {
            // note: using ">" in return statements means that smaller element appears before larger.
            if (a->numVariables() != b->numVariables()) {
                // prefer node with less variables
                return a->numVariables() > b->numVariables();
            }
            if(a->numNeighbors() != b->numNeighbors()) {
                // prefer node with less neighbors
                return a->numNeighbors() > b->numNeighbors();
            }
            return a<b;
        }
    };

    // represents a possible step in the query pipeline
    struct QueryPipelineNode
    {
        explicit QueryPipelineNode(const DependencyNodePtr &node)
        : node_(node)
        {
            // add all nodes to a priority queue
            for(auto &neighbor : node->neighbors()) {
                neighbors_.push(neighbor);
            }
        }
        const DependencyNodePtr node_;
        std::priority_queue<DependencyNodePtr, std::vector<DependencyNodePtr>, CompareNodes> neighbors_;
    };
    using QueryPipelineNodePtr = std::shared_ptr<QueryPipelineNode>;

    class AnswerBuffer_WithReference : public AnswerBuffer {
    public:
        explicit AnswerBuffer_WithReference(const std::shared_ptr<QueryPipeline> &pipeline)
        : AnswerBuffer(), pipeline_(pipeline) {}
    protected:
        std::shared_ptr<QueryPipeline> pipeline_;
    };
}

KnowledgeBase::KnowledgeBase(const boost::property_tree::ptree &config)
: threadPool_(std::make_shared<ThreadPool>(std::thread::hardware_concurrency()))
{
	backendManager_ = std::make_shared<KnowledgeGraphManager>(threadPool_);
	reasonerManager_ = std::make_shared<ReasonerManager>(threadPool_, backendManager_);
	loadConfiguration(config);
}

KnowledgeBase::KnowledgeBase(const std::string_view &configFile)
: threadPool_(std::make_shared<ThreadPool>(std::thread::hardware_concurrency()))
{
	backendManager_ = std::make_shared<KnowledgeGraphManager>(threadPool_);
	reasonerManager_ = std::make_shared<ReasonerManager>(threadPool_, backendManager_);

	boost::property_tree::ptree config;
	boost::property_tree::read_json(URI::resolve(configFile), config);
	loadConfiguration(config);
}

void KnowledgeBase::loadConfiguration(const boost::property_tree::ptree &config)
{
    auto semwebTree = config.get_child_optional("semantic-web");
    if(semwebTree) {
        // load RDF URI aliases
        auto prefixesList = semwebTree.value().get_child_optional("prefixes");
        for(const auto &pair : prefixesList.value()) {
            auto alias = pair.second.get("alias","");
            auto uri = pair.second.get("uri","");
            if(!alias.empty() && !uri.empty()) {
                semweb::PrefixRegistry::get().registerPrefix(alias, uri);
            }
            else {
                KB_WARN("Invalid entry in semantic-web::prefixes, 'alias' and 'uri' must be defined.");
            }
        }
    }

    // initialize RDF data backends from configuration
	auto backendList = config.get_child_optional("data-backends");
	if(backendList) {
		for(const auto &pair : backendList.value()) {
			try {
                backendManager_->loadKnowledgeGraph(pair.second);
			}
			catch(std::exception& e) {
				KB_ERROR("failed to load a knowledgeGraph: {}", e.what());
			}
		}
	}
	else {
		KB_ERROR("configuration has no 'backends' key.");
	}

	auto reasonerList = config.get_child_optional("reasoner");
	if(reasonerList) {
		for(const auto &pair : reasonerList.value()) {
			try {
				reasonerManager_->loadReasoner(pair.second);
			}
			catch(std::exception& e) {
				KB_ERROR("failed to load a reasoner: {}", e.what());
			}
		}
	}
	else {
		KB_ERROR("configuration has no 'reasoner' key.");
	}

	auto dataSourcesList = config.get_child_optional("data-sources");
	if(dataSourcesList) {
	    static const std::string formatDefault = {};

		for(const auto &pair : dataSourcesList.value()) {
			auto &subtree = pair.second;
			auto dataFormat = subtree.get("format",formatDefault);
			auto source = std::make_shared<DataSource>(dataFormat);
			source->loadSettings(subtree);

			for(auto &kg_pair : backendManager_->knowledgeGraphPool()) {
			    // FIXME: handle format specified in settings file
			    kg_pair.second->knowledgeGraph()->loadFile(source->uri(), TripleFormat::RDF_XML);
			}
        }
    }
}

std::shared_ptr<KnowledgeGraph> KnowledgeBase::centralKG()
{
    auto knowledgeGraphs_ = backendManager_->knowledgeGraphPool();
    std::shared_ptr<KnowledgeGraph> centralKG;
    if(knowledgeGraphs_.empty()) {
        return {};
    }
    else {
        // TODO: flag one of the KGs as "preferred" in settings
        return (*knowledgeGraphs_.begin()).second->knowledgeGraph();
    }
}

std::vector<RDFComputablePtr> KnowledgeBase::createComputationSequence(
        const std::list<DependencyNodePtr> &dependencyGroup)
{
    // Pick a node to start with.
    // For now the one with minimum number of neighbors is picked.
    DependencyNodePtr first;
    unsigned long minNumNeighbors = 0;
    for(auto &n : dependencyGroup) {
        if(!first || n->numNeighbors() < minNumNeighbors) {
            minNumNeighbors = n->numNeighbors();
            first = n;
        }
    }

    // remember visited nodes, needed for circular dependencies
    // all nodes added to the queue should also be added to this set.
    std::set<DependencyNode*> visited;
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
    while(!queue.empty()) {
        auto front = queue.front();

        // get top successor node that has not been visited yet
        DependencyNodePtr topNext;
        while(!front->neighbors_.empty()) {
            auto topNeighbor = front->neighbors_.top();
            front->neighbors_.pop();

            if(visited.count(topNeighbor.get()) == 0) {
                topNext = topNeighbor;
                break;
            }
        }
        // pop element from queue if all neighbors were processed
        if(front->neighbors_.empty()) {
            queue.pop_front();
        }

        if(topNext) {
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
    const std::shared_ptr<AnswerBroadcaster> &pipelineInput,
    const std::shared_ptr<AnswerBroadcaster> &pipelineOutput,
    const QueryContextPtr &ctx)
{
    // This function generates a query pipeline for literals that
    // can be computed (EDB-only literals are processed separately).
    // The literals are part of one dependency group.
    // They are sorted, and also evaluated in this order.
    // For each computable literal there is at least one reasoner
    // that can compute the literal.
    // However, instances of the literal may also occur in the EDB.
    // Hence, computation results must be combined with results of
    // an EDB query for each literal.
    // There are more sophisticated strategies that could be employed
    // to reduce number of EDB calls.
    // For the moment we rather use a simple implementation
    // where a parallel query step is created for each computable literal.
    // In this step, an EDB query and computations run in parallel.
    // The results are then given to the next step if any.

    auto lastOut = pipelineInput;
    auto edb = centralKG();

    for(auto &lit : computableLiterals) {
        auto stepInput = lastOut;
        auto stepOutput = std::make_shared<AnswerBroadcaster>();
        pipeline->addStage(stepOutput);

        auto edbStage = std::make_shared<EDBStage>(edb, lit, ctx);
        edbStage->selfWeakRef_ = edbStage;
        stepInput >> edbStage;
        edbStage >> stepOutput;
        pipeline->addStage(edbStage);

        for(auto &r : lit->reasonerList()) {
            auto idbStage = std::make_shared<IDBStage>(r, lit, threadPool_, ctx);
            idbStage->selfWeakRef_ = idbStage;
            stepInput >> idbStage;
            idbStage >> stepOutput;
            pipeline->addStage(idbStage);
        }

        lastOut = stepOutput;
    }

    lastOut >> pipelineOutput;
}

AnswerBufferPtr KnowledgeBase::submitQuery(const GraphQueryPtr &graphQuery)
{
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
    for(auto &l : allLiterals) {
        if(l->isNegated()) negativeLiterals.push_back(l);
        else               positiveLiterals.push_back(l);
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
    for(auto &l : positiveLiterals) {
        std::vector<std::shared_ptr<Reasoner>> l_reasoner;
        for(auto &pair : reasonerManager_->reasonerPool()) {
            auto &r = pair.second->reasoner();
            if(r->canEvaluate(*l)) l_reasoner.push_back(r);
        }
        if(l_reasoner.empty()) edbOnlyLiterals.push_back(l);
        else computableLiterals.push_back(std::make_shared<RDFComputable>(*l, l_reasoner));
    }

    // --------------------------------------
    // run EDB query with all edb-only literals.
    // --------------------------------------
    std::shared_ptr<AnswerBuffer> edbOut;
    if(edbOnlyLiterals.empty()) {
        edbOut = std::make_shared<AnswerBuffer>();
        auto channel = AnswerStream::Channel::create(edbOut);
        channel->push(AnswerStream::bos());
        channel->push(AnswerStream::eos());
    }
    else {
        auto edbOnlyQuery = std::make_shared<GraphQuery>(
                    edbOnlyLiterals, graphQuery->ctx());
        edbOut = kg->submitQuery(edbOnlyQuery);
    }
    pipeline->addStage(edbOut);

    // --------------------------------------
    // handle positive IDB literals.
    // --------------------------------------
    std::shared_ptr<AnswerBroadcaster> idbOut;
    if(computableLiterals.empty())
    {
        idbOut = edbOut;
    }
    else
    {
        idbOut = std::make_shared<AnswerBroadcaster>();
        pipeline->addStage(idbOut);
        // --------------------------------------
        // Compute dependency groups of computable literals.
        // --------------------------------------
        DependencyGraph dg;
        dg.insert(computableLiterals.begin(), computableLiterals.end());

        // --------------------------------------
        // Construct a pipeline for each dependency group.
        // --------------------------------------
        if(dg.numGroups()==1) {
            auto &literalGroup = *dg.begin();
            createComputationPipeline(
                    pipeline,
                    createComputationSequence(literalGroup.member_),
                    edbOut,
                    idbOut,
                    graphQuery->ctx());
        }
        else {
            // there are multiple dependency groups. They can be evaluated in parallel.

            // combines answers computed in different parallel steps
            auto answerCombiner = std::make_shared<AnswerCombiner>();
            // create a parallel step for each dependency group
            for(auto &literalGroup : dg)
            {
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
    std::shared_ptr<AnswerBroadcaster> lastStage;
    if(!negativeLiterals.empty()) {
        // run a dedicated stage where negated literals can be evaluated in parallel
        auto negStage = std::make_shared<LiteralNegationStage>(
                this, graphQuery->ctx(), negativeLiterals);
        idbOut >> negStage;
        lastStage = negStage;
    }
    else {
        lastStage = idbOut;
    }

    /*
            // optionally add a stage to the pipeline that drops all redundant result.
            if(graphQuery->flags() & QUERY_FLAG_UNIQUE_SOLUTIONS) {
                auto filterStage = std::make_shared<RedundantAnswerFilter>();
                lastStage >> filterStage;
                lastStage = filterStage;
            }

            // TODO: optionally persist top-down inferences in data backend(s)
            if(graphQuery->flags() & QUERY_FLAG_PERSIST_SOLUTIONS) {
                //auto persistStage = std::make_shared<XXX>();
                //lastStage >> persistStage;
                //lastStage = persistStage;
            }
    */

    auto out = std::make_shared<AnswerBuffer_WithReference>(pipeline);
    lastStage >> out;
    edbOut->stopBuffering();
    return out;
}

AnswerBufferPtr KnowledgeBase::submitQuery(const LiteralPtr &literal, const QueryContextPtr &ctx)
{
    auto rdfLiteral = RDFLiteral::fromLiteral(literal, ctx->selector_);
    return submitQuery(std::make_shared<GraphQuery>(
        GraphQuery({rdfLiteral}, ctx)));
}

AnswerBufferPtr KnowledgeBase::submitQuery(const FormulaPtr &phi, const QueryContextPtr &ctx)
{
    auto outStream = std::make_shared<AnswerBuffer>();

    auto pipeline = std::make_shared<QueryPipeline>();
    pipeline->addStage(outStream);

    // --------------------------------------
    // Pick a Knowledge Graph for EDB queries
    // --------------------------------------
    std::shared_ptr<KnowledgeGraph> kg = centralKG();

    // decompose input formula into parts that are considered in disjunction,
    // and thus can be evaluated in parallel.
    QueryTree qt(phi);
    for(auto &path : qt)
    {
    	// each node in a path is either a predicate, a negated predicate,
		// a modal formula, or the negation of a modal formula.

		std::vector<RDFLiteralPtr> posLiterals, negLiterals;
		std::vector<std::shared_ptr<ModalFormula>> posModals, negModals;

        for(auto &node : path.nodes()) {
			switch(node->type()) {
			case FormulaType::PREDICATE:
				// TODO: rather directly construct RDFLit?
				posLiterals.push_back(RDFLiteral::fromLiteral(std::make_shared<Literal>(
					std::static_pointer_cast<Predicate>(node),
					false
				),  ctx->selector_));
				break;

			case FormulaType::MODAL:
				posModals.push_back(std::static_pointer_cast<ModalFormula>(node));
				break;

			case FormulaType::NEGATION: {
				auto negation = (Negation*)node.get();
				auto negated = negation->negatedFormula();
				switch(negated->type()) {
				case FormulaType::PREDICATE:
					// TODO: rather directly construct RDFLit?
					negLiterals.push_back(RDFLiteral::fromLiteral(std::make_shared<Literal>(
						std::static_pointer_cast<Predicate>(negated), true), ctx->selector_));
					break;
				case FormulaType::MODAL:
					negModals.push_back(std::static_pointer_cast<ModalFormula>(negated));
					break;
				default:
					throw QueryError("Unexpected negated formula type {} in QueryTree.", (int)negated->type());
				}
				break;
			}
			default:
				throw QueryError("Unexpected formula type {} in QueryTree.", (int)node->type());
			}
		}

		std::shared_ptr<AnswerBroadcaster> lastStage;
		std::shared_ptr<AnswerBuffer> firstBuffer;

		// first evaluate positive literals if any
		if(posLiterals.empty()) {
			// if there are none, we still need to indicate begin and end of stream for the rest of the pipeline.
			// so we just push `bos` (an empty substitution) followed by `eos` and feed these messages to the next stage.
			// FIXME: need to call stopBuffering!?!
			firstBuffer = std::make_shared<AnswerBuffer>();
			lastStage = firstBuffer;
			auto channel = AnswerStream::Channel::create(lastStage);
			channel->push(AnswerStream::bos());
			channel->push(AnswerStream::eos());
        	pipeline->addStage(lastStage);
		}
		else {
        	auto pathQuery = std::make_shared<GraphQuery>(posLiterals, ctx);
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
		for(auto &posModal : posModals) {
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
		if(!negLiterals.empty()) {
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
		if(!negModals.empty()) {
			// run a dedicated stage where negated modals can be evaluated in parallel
			auto negModalStage = std::make_shared<ModalNegationStage>(
					this, ctx, negModals);
			lastStage >> negModalStage;
			lastStage = negModalStage;
        	pipeline->addStage(lastStage);
		}

		/*
        auto &literals = path.literals();
        std::vector<RDFLiteralPtr> rdfLiterals(literals.size());
        uint32_t literalIndex=0;
        for(auto &l : literals) {
            rdfLiterals[literalIndex++] = RDFLiteral::fromLiteral(l);
        }
        auto pathQuery = std::make_shared<GraphQuery>(rdfLiterals, queryFlags);
        */

        lastStage >> outStream;
        firstBuffer->stopBuffering();
        pipeline->addStage(lastStage);
    }

    auto out = std::make_shared<AnswerBuffer_WithReference>(pipeline);
    outStream >> out;
    outStream->stopBuffering();
    return out;
}

bool KnowledgeBase::insert(const std::vector<StatementData> &propositions)
{
    bool status = true;
    for(auto &kg : backendManager_->knowledgeGraphPool()) {
        if(!kg.second->knowledgeGraph()->insert(propositions)) {
            KB_WARN("assertion of triple data failed!");
            status = false;
        }
    }
    return status;
}

bool KnowledgeBase::insert(const StatementData &proposition)
{
    bool status = true;
    // assert each statement into each knowledge graph backend
    for(auto &kg : backendManager_->knowledgeGraphPool()) {
        if(!kg.second->knowledgeGraph()->insert(proposition)) {
            KB_WARN("assertion of triple data failed!");
            status = false;
        }
    }
    return status;
}
