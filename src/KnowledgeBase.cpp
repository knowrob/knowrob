/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <thread>

#include <knowrob/Logger.h>
#include <knowrob/KnowledgeBase.h>
#include "knowrob/semweb/PrefixRegistry.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/queries/QueryTree.h"
#include "knowrob/queries/AnswerCombiner.h"
#include "knowrob/queries/IDBStage.h"
#include "knowrob/queries/EDBStage.h"

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
        explicit AnswerBuffer_WithReference(const std::shared_ptr<AnswerStream> &referencedStream)
        : AnswerBuffer(), referencedStream_(referencedStream) {}
    protected:
        std::shared_ptr<AnswerStream> referencedStream_;
    };
}

KnowledgeBase::KnowledgeBase(const boost::property_tree::ptree &config)
: threadPool_(std::make_shared<ThreadPool>(std::thread::hardware_concurrency()))
{
	backendManager_ = std::make_shared<KnowledgeGraphManager>(threadPool_);
	reasonerManager_ = std::make_shared<ReasonerManager>(threadPool_, backendManager_);
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

AnswerBufferPtr KnowledgeBase::submitQuery(const FormulaPtr &phi, int queryFlags)
{
    auto outStream = std::make_shared<AnswerBuffer>();

    // convert into DNF and submit a query for each conjunction.
    // note that the number of conjunctions can get pretty high for
    // queries using several disjunctions.
    // e.g. (p|~q)&(r|q) would create 4 paths (p&r, p&q, ~q&r and ~q&q).
    // it is assumed here that queries are rather simple and that
    // the number of path's in the query tree is rather low.
    QueryTree qt(phi);
    for(auto &path : qt)
    {
        auto &literals = path.literals();
        std::vector<RDFLiteralPtr> rdfLiterals(literals.size());
        uint32_t literalIndex=0;
        for(auto &l : literals) {
            rdfLiterals[literalIndex++] = RDFLiteral::fromLiteral(l);
        }
        auto pathQuery = std::make_shared<GraphQuery>(rdfLiterals, queryFlags);

        auto pathOutput = submitQuery(pathQuery);
        pathOutput >> outStream;
        pathOutput->stopBuffering();
    }

    outStream->stopBuffering();
    return outStream;
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
    const std::vector<RDFComputablePtr> &computableLiterals,
    const std::shared_ptr<AnswerBroadcaster> &pipelineInput,
    const std::shared_ptr<AnswerBroadcaster> &pipelineOutput,
    int queryFlags)
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

        auto edbStage = std::make_shared<EDBStage>(edb, lit, queryFlags);
        stepInput >> edbStage;
        edbStage >> stepOutput;

        for(auto &r : lit->reasonerList()) {
            auto idbStage = std::make_shared<IDBStage>(r, lit, threadPool_, queryFlags);
            stepInput >> idbStage;
            idbStage >> stepOutput;
        }

        lastOut = stepOutput;
    }

    lastOut >> pipelineOutput;
}

AnswerBufferPtr KnowledgeBase::submitQuery(const GraphQueryPtr &graphQuery)
{
    auto allLiterals = graphQuery->literals();

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

    std::shared_ptr<AnswerBuffer> edbOut;
    // --------------------------------------
    // run EDB query with all edb-only literals.
    // --------------------------------------

    if(edbOnlyLiterals.empty()) {
        edbOut = std::make_shared<AnswerBuffer>();
        auto channel = AnswerStream::Channel::create(edbOut);
        channel->push(AnswerStream::bos());
        channel->push(AnswerStream::eos());
    }
    else {
        auto edbOnlyQuery = std::make_shared<GraphQuery>(
                    edbOnlyLiterals,
                    graphQuery->flags());
        edbOut = kg->submitQuery(edbOnlyQuery);
    }

    std::shared_ptr<AnswerBroadcaster> idbOut;
    if(computableLiterals.empty())
    {
        idbOut = edbOut;
    }
    else
    {
        idbOut = std::make_shared<AnswerBroadcaster>();
        // --------------------------------------
        // Compute dependency groups of computable literals.
        // --------------------------------------
        DependencyGraph dg;
        dg.insert(computableLiterals.begin(), computableLiterals.end());

        if(dg.numGroups()==1) {
            auto &literalGroup = *dg.begin();
            // --------------------------------------
            // Construct a pipeline for each dependency group.
            // --------------------------------------
            createComputationPipeline(
                createComputationSequence(literalGroup.member_),
                edbOut,
                idbOut,
                graphQuery->flags());
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
                        createComputationSequence(literalGroup.member_),
                        edbOut,
                        answerCombiner,
                        graphQuery->flags());
            }
            answerCombiner >> idbOut;
        }
    }

    // --------------------------------------
    // Evaluate all negative literals in parallel.
    // --------------------------------------
    std::shared_ptr<AnswerBroadcaster> lastStage;
    if(negativeLiterals.empty()) {
        lastStage = idbOut;
    }
    else {
        // append a parallel step for each negative literal and reasoner that can compute it.
        // FIXME: maybe every possible source must report that a negated literal cannot be grounded?
        //xxx;
        KB_WARN("todo: submitQuery negations");
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

    // TODO: It might be better to rather use `<<` for streaming than `>>`, and to hold a reference of input in output
    //       stream assuming the output stream is what the user will work with!
    auto out = std::make_shared<AnswerBuffer_WithReference>(edbOut);
    lastStage >> out;
    edbOut->stopBuffering();
    return out;
}

AnswerBufferPtr KnowledgeBase::submitQuery(const LiteralPtr &literal, int queryFlags)
{
    auto rdfLiteral = RDFLiteral::fromLiteral(literal);
    return submitQuery(std::make_shared<GraphQuery>(
        GraphQuery({rdfLiteral}, queryFlags)));
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
