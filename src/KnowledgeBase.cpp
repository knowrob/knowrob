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
#include "knowrob/reasoner/Blackboard.h"
#include "knowrob/semweb/PrefixRegistry.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/queries/QueryTree.h"
#include "knowrob/queries/AnswerCombiner.h"

using namespace knowrob;

namespace knowrob {
    struct GraphQueryData {
        std::shared_ptr<GraphQuery> graphQuery_;
        std::shared_ptr<AnswerBroadcaster> outputStream_;
    };

    class ReasonerRunner : public ThreadPool::Runner {
    public:
        std::shared_ptr<IReasoner> reasoner_;
        std::shared_ptr<AllocatedQuery> query_;

        ReasonerRunner(const std::shared_ptr<IReasoner> &reasoner,
                       const std::shared_ptr<GraphQueryData> &queryData)
        : reasoner_(reasoner), ThreadPool::Runner()
        {
            auto outputChannel = AnswerStream::Channel::create(queryData->outputStream_);
            auto literals = queryData->graphQuery_->literals();
            // TODO: at least need to group into negative and positive literals here.
            //       also would make sense to sort according to number of variables
            auto phi = std::make_shared<Conjunction>(literals);
            auto q = std::make_shared<Query>(
                    phi,
                    queryData->graphQuery_->flags(),
                    queryData->graphQuery_->modalFrame());
            query_ = std::make_shared<AllocatedQuery>(q, outputChannel);
        }

        void run() override {
            reasoner_->runQuery(query_);
        }
    };

    class AnswerBuffer_WithPipelines : public AnswerBuffer {
    public:
        AnswerBuffer_WithPipelines() : AnswerBuffer() {}
        std::list<QueryPipeline> pipelines_;
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
}

AnswerBufferPtr KnowledgeBase::submitQuery(const FormulaPtr &phi, int queryFlags)
{
    auto outStream = std::make_shared<AnswerBuffer_WithPipelines>();

    // convert into DNF and submit a query for each conjunction.
    // note that the number of conjunctions can get pretty high for
    // queries using several disjunctions.
    // e.g. (p|~q)&(r|q) would create 4 paths (p&r, p&q, ~q&r and ~q&q).
    // it is assumed here that queries are rather simple and that
    // the number of path's in the query tree is rather low.
    QueryTree qt(phi);
    for(auto &path : qt)
    {
        // group literals by modality
        std::map<ModalityLabelPtr, std::list<LiteralPtr>> modalityMap;
        for(auto &labeled : path.literals()) {
            auto label = std::static_pointer_cast<ModalityLabel>(labeled->label());
            auto it = modalityMap.find(label);
            if(it == modalityMap.end()) {
                modalityMap[label] = { labeled };
            }
            else {
                it->second.push_back(labeled);
            }
        }

        // compute dependencies
        DependencyGraph dg;
        for(auto &pair : modalityMap) {
            dg.insert(pair.second, pair.first);
        }

        // create a query pipeline for each conjunction.
        // NOTE: here the output stream keeps a reference on each pipeline feeding it.
        //       the idea is that as long as outStream is used, the pipeline will be alive.
        //       keeping references in the KB instead would require additional bookkeeping for
        //       removing finished pipelines.
        auto &pipeline = outStream->pipelines_.emplace_back(outStream);
        pipeline.setQueryEngine(this);
        pipeline.setQueryFlags(queryFlags);
        // iterate over groups of modal queries with shared free variables
        // and create a query pipeline where each group is processed in parallel steps.
        for(auto &queryGroup : dg) {
            pipeline.addDependencyGroup(queryGroup.member_);
        }
        pipeline.run();
    }

    return outStream;
}

AnswerBufferPtr KnowledgeBase::submitQuery(const GraphQueryPtr &graphQuery)

//const std::vector<LiteralPtr> &literals,
                                           //const ModalityLabelPtr &label,
                                         //  int queryFlags)
{
    std::list<std::shared_ptr<IReasoner>> tdReasoner;
    for(auto &pair : reasonerManager_->reasonerPool()) {
        auto &r = pair.second->reasoner();
        // skip reasoner that cannot perform top-down query evaluation
        if(!r->hasCapability(CAPABILITY_TOP_DOWN_EVALUATION)) continue;
        // TODO: consider filtering based on whether the reasoner has a definition for one of the literals.
        tdReasoner.push_back(r);
    }

    if(tdReasoner.empty()) {
        // just run EDB query in case no top-down reasoner is in use.
        auto knowledgeGraphs_ = backendManager_->knowledgeGraphPool();
        if(knowledgeGraphs_.empty()) {
            return {};
        }
        else {
            // TODO: flag one of the KGs as "preferred" in settings
            auto preferredKnowledgeGraph = (*knowledgeGraphs_.begin()).second->knowledgeGraph();
            return preferredKnowledgeGraph->submitQuery(graphQuery);
        }
    }

    // compute dependency groups based on shared variables of literals.
    // each group can be evaluated independently.
    DependencyGraph dg;
    for(auto &literal : graphQuery->literals()) {
        dg.insert(literal);
    }

    // TODO: use AnswerCombiner without unification. the solutions that are combined do not have shared variables.
    // NOTE: it is assumed that top-down reasoner will include all EDB facts in their set of answers.
    auto answerCombiner = std::make_shared<AnswerCombiner>();

    // add additional stages for post-processing of results
    std::shared_ptr<AnswerBroadcaster> lastStage = answerCombiner;
    {
        // TODO: optionally add a stage to the pipeline that drops all redundant result.
        if(graphQuery->flags() & QUERY_FLAG_UNIQUE_SOLUTIONS) {
            /*
            auto filterStage = std::make_shared<XXX>();
            lastStage >> filterStage;
            lastStage = filterStage;
            */
        }

        // TODO: notify top-down reasoner about inferences of others.
        if(tdReasoner.size()>1) {
            /*
            auto notifyStage = std::make_shared<XXX>();
            lastStage >> notifyStage;
            lastStage = notifyStage;
            */
        }

        // TODO: optionally persist top-down inferences in data backend(s)
        if(graphQuery->flags() & QUERY_FLAG_PERSIST_SOLUTIONS) {
            /*
            auto persistStage = std::make_shared<XXX>();
            lastStage >> persistStage;
            lastStage = persistStage;
            */
        }
    }
    auto out = std::make_shared<AnswerBuffer>();
    lastStage >> out;

    // iterate over dependency groups, and create reasoner runner writing into answerCombiner stream
    for(auto &literalGroup : dg)
    {
        // for a group each reasoner writes into the same stream which in turn is streamed into
        // the stream combining results from different groups.
        auto groupAnswers = std::make_shared<AnswerBroadcaster>();
        groupAnswers >> answerCombiner;

        std::vector<LiteralPtr> queryLiterals;
        for(auto &x : literalGroup.member_) {
            std::shared_ptr<LiteralDependencyNode> literalNode;
            if((literalNode = std::dynamic_pointer_cast<LiteralDependencyNode>(x))) {
                queryLiterals.push_back(literalNode->literal());
            }
        }

        auto queryData = std::make_shared<GraphQueryData>();
        queryData->graphQuery_ = std::make_shared<GraphQuery>(
                queryLiterals,
                graphQuery->flags(),
                graphQuery->modalFrame());
        queryData->outputStream_ = groupAnswers;

        std::list<std::shared_ptr<ReasonerRunner>> runnerList;
        // for each group and top-down reasoner, run a graph query via a worker thread.
        // each reasoner writes into an individual channel of the stream.
        // note: below splits into two for-loops such that all channels are created first to avoid race condition with
        //       worker threads finishing before all channels are created.
        for(auto &r : tdReasoner) {
            runnerList.push_back(std::make_shared<ReasonerRunner>(r, queryData));
        }
        for(auto &runner : runnerList) {
            threadPool_->pushWork(runner);
        }
    }

    return out;
}

AnswerBufferPtr KnowledgeBase::submitQuery(const LiteralPtr &query, int queryFlags)
{
    return submitQuery(std::make_shared<GraphQuery>(
        GraphQuery({query}, queryFlags)));
}

AnswerBufferPtr KnowledgeBase::submitQuery(const LabeledLiteralPtr &query, int queryFlags)
{
    auto &modalOperators = query->label()->modalOperators();
    return submitQuery(std::make_shared<GraphQuery>(
        GraphQuery({query}, queryFlags, ModalFrame(modalOperators))));
}

bool KnowledgeBase::insert(const std::vector<TripleData> &statements)
{
    bool allProjected = true;
    for(auto &x : statements) {
        allProjected = insert(x) && allProjected;
    }
    return allProjected;
}

bool KnowledgeBase::insert(const TripleData &statement)
{
    bool status = true;
    // assert each statement into each knowledge graph backend
    for(auto &kg : backendManager_->knowledgeGraphPool()) {
        if(!kg.second->knowledgeGraph()->insert(statement)) {
            KB_WARN("assertion of triple data failed!");
            status = false;
        }
    }
    return status;
}


void KnowledgeBase::runQuery(const std::shared_ptr<const Query> &query, QueryResultHandler &handler)
{
	auto bbq = std::make_shared<knowrob::AnswerQueue>();
	auto bb = std::make_shared<Blackboard>(reasonerManager_.get(), bbq, query);
	AnswerPtr solution;

	bb->start();
	do {
		solution = bbq->pop_front();
		if(AnswerStream::isEOS(solution)) {
			break;
		}
	} while(handler.pushQueryResult(solution));
}
