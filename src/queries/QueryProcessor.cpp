//
// Created by daniel on 26.03.23.
//

#include "knowrob/Logger.h"
#include "knowrob/queries/QueryProcessor.h"
#include "knowrob/queries/QueryTree.h"
#include "knowrob/queries/DependencyGraph.h"

using namespace knowrob;

QueryProcessor::QueryProcessor(const KnowledgeBasePtr &kb)
: kb_(kb)
{
}

std::shared_ptr<QueryResultQueue> QueryProcessor::operator<<(const FormulaPtr &phi)
{
    auto outQueue = std::make_shared<knowrob::QueryResultQueue>();

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
        auto &pipeline = pipelines_.emplace_back(outQueue);
        for(auto &queryGroup : dg) {
            pipeline.addDependencyGroup(queryGroup.member_);
        }
        pipeline.run();
    }

    return outQueue;
}

std::shared_ptr<QueryResultQueue> QueryProcessor::operator<<(const LiteralPtr &literal)
{
    auto outQueue = std::make_shared<knowrob::QueryResultQueue>();
    // TODO: just submit a graph query here
    return outQueue;
}

std::shared_ptr<QueryResultQueue> QueryProcessor::operator<<(const LabeledLiteralPtr &literal)
{
    auto outQueue = std::make_shared<knowrob::QueryResultQueue>();
    // TODO: just submit a graph query here
    return outQueue;
}
