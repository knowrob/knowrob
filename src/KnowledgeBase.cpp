/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/Logger.h>
#include <knowrob/KnowledgeBase.h>
#include "knowrob/reasoner/Blackboard.h"
#include "knowrob/semweb/PrefixRegistry.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/queries/QueryTree.h"

using namespace knowrob;

KnowledgeBase::KnowledgeBase(const boost::property_tree::ptree &config)
{
	reasonerManager_ = std::make_shared<ReasonerManager>();
	loadConfiguration(config);
	if(!knowledgeGraph_) {
	    // TODO: rather fallback to default implementation
	    throw std::runtime_error("knowledge graph must be configured");
	}
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

BufferedAnswersPtr KnowledgeBase::submitQuery(const FormulaPtr &phi, int queryFlags)
{
    // TODO: handle flags
    auto outStream = std::make_shared<BufferedAnswers>();

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

        // iterate over groups of modal queries with shared free variables
        // and create a query pipeline where each group is processed
        // in parallel steps.
        // FIXME: need a mechanism to remove finished pipelines from the pipelines_ list!
        auto &pipeline = pipelines_.emplace_back(outStream);
        pipeline.setQueryEngine(this);
        pipeline.setQueryFlags(queryFlags);
        for(auto &queryGroup : dg) {
            pipeline.addModalGroup(queryGroup.member_);
        }
        pipeline.run();
    }

    return outStream;
}

BufferedAnswersPtr KnowledgeBase::submitQuery(const LiteralPtr &query, int queryFlags)
{
    return submitQuery({query}, {}, queryFlags);
}

BufferedAnswersPtr KnowledgeBase::submitQuery(const LabeledLiteralPtr &query, int queryFlags)
{
    return submitQuery(
        {query},
        std::dynamic_pointer_cast<ModalityLabel>(query->label()),
        queryFlags);
}

BufferedAnswersPtr KnowledgeBase::submitQuery(const std::vector<LiteralPtr> &literals,
                                              const ModalityLabelPtr &label,
                                              int queryFlags)
{
    // TODO: come up with a plan how labels can be handled.
    // NOTE: the literals are assumed to be member of the same dependency group here.
    //       so for every literal there is another literal in the vector with a shared
    //       free variable.

    // TODO: run a query with extensional data stored in knowledgeGraph_
    //      - design and implement KG interface
    //          - currently BufferedAnswersPtr KnowledgeGraph::submitQuery(const GraphQueryPtr &query)
    //          - but mongo has mongo::AnswerCursorPtr lookupTriplePaths(const std::vector<semweb::TripleExpression>&)
    //      - XXX: so how to hook this into a pipeline?
    //          - well maybe not needed. maybe could stream both into the same buffered stream
    //            and return this

    // compute dependencies
    DependencyGraph dg;
    for(auto &literal : literals) dg.insert(literal);
    // iterate over dependency groups
    for(auto &literalGroup : dg) {
        // TODO: create a pipeline for top-down reasoner
        //      - build a pipeline that runs each group in parallel,
        //       and a simple combiner node without unification to combine results of groups.
        //      - reuse Blackboard code
        //          - merge nodes into complex queries if reasoner supports it and defines
        //            the predicate
        // TODO: for each group first pick literal with max priority
        //      - prefer edges with more grounded nodes
        // TODO: then continue expanding path according to preference and neighbor relation
    }

    // TODO: allow storing inference results in KG
    // TODO: avoid redundant results
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

bool KnowledgeBase::projectIntoEDB(const std::list<Statement> &statements, const std::string &reasonerID)
{
    bool allProjected = true;
    for(auto &x : statements) {
        allProjected = projectIntoEDB(x, reasonerID) && allProjected;
    }
    return allProjected;
}

bool KnowledgeBase::projectIntoEDB(const Statement &statement, const std::string &reasonerID)
{
    if(reasonerID == "*") {
        bool oneSucceeded = false;
        bool oneAttempted = false;
        for(auto &pair : reasonerManager_->reasonerPool()) {
            if(pair.second->reasoner()->hasCapability(CAPABILITY_DYNAMIC_ASSERTIONS)) {
                auto description_n = pair.second->reasoner()->getPredicateDescription(statement.predicate()->indicator());
                if(description_n) {
                    oneSucceeded = pair.second->reasoner()->projectIntoEDB(statement) || oneSucceeded;
                    oneAttempted = true;
                }
            }
        }
        if(!oneSucceeded && !oneAttempted) {
            KB_ERROR("None of the known reasoners is able to project the predicate {}.", *statement.predicate());
        }
        return oneSucceeded;
    }
    else {
        // project into user-specified reasoner backend
        auto definedReasoner = reasonerManager_->getReasonerWithID(reasonerID);
        if(definedReasoner) {
            if(definedReasoner->reasoner()->hasCapability(CAPABILITY_DYNAMIC_ASSERTIONS)) {
                return definedReasoner->reasoner()->projectIntoEDB(statement);
            }
            else {
                KB_ERROR("Reasoner with id '{}' has not the capability to dynamically assert factual knowledge.", reasonerID);
            }
        } else {
            KB_ERROR("No reasoner with id '{}' is known.", reasonerID);
        }
        return false;
    }
}
