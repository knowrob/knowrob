/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_FACT_BASE_H__
#define __KNOWROB_FACT_BASE_H__

#include <set>

#include <knowrob/db/IDataSource.h>
#include <knowrob/lang/terms.h>

namespace knowrob {
    /**
     * A collection of facts. Each fact is represented as a n-ary predicate.
     * This abstract interface does not support sophisticated querying.
     * But particular implementations may support more sophisticated querying, e.g.
     * the aggregation framework in case of data managed by MongoDB.
     */
    class IFactBase : public IDataSource {
    public:
        virtual ~IFactBase(){}

        /** Get list of EDB predicates in this fact base.
         *
         * @return list of predicate indicators
         */
        const std::set<PredicateIndicator>& predicateIndicators() const
            { return predicateIndicators_; }

        /** Get whether the fact base stores a particular predicate.
         *
         * @param indicator a predicate indicator
         * @return true if this data source has an entry for the given predicate indicator
         */
        bool containsPredicate(const PredicateIndicator &indicator) const
            { return predicateIndicators_.find(indicator) != predicateIndicators_.end(); }

        /** Read all facts currently stored in this data source and publish them via a message queue.
         * The function is supposed to return _after_ the queue has been filled, a EOS message is
         * appended automatically after returning.
         *
         * @param msgQueue a queue of facts
         */
        virtual void readFacts(QueryResultQueue &msgQueue) const = 0;

        /** Read all facts of one predicate currently stored in this data source and publish them via a message queue.
         * The function is supposed to return _after_ the queue has been filled, a EOS message is
         * appended automatically after returning.
         *
         * @param msgQueue a queue of facts
         * @param indicator a predicate indicator
         */
        virtual void readFacts(QueryResultQueue &msgQueue, const PredicateIndicator &indicator) const = 0;

        // todo: add an interface for watching the DB, e.g. would be needed to keep PrologReasoner in synch with
        //        external database. But some databases may not support this, or are by definition read-only etc.
        //virtual void watch(std::shared_ptr<MessageQueue<Operation<FactType>>> &msgQueue) = 0;
        //virtual void watch(std::shared_ptr<MessageQueue<Operation<FactType>>> &msgQueue, const PredicateIndicator &prediacte) = 0;

    protected:
        std::set<PredicateIndicator> predicateIndicators_;
    };
}

#endif //__KNOWROB_FACT_BASE_H__
