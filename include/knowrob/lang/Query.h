/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_QUERY_H__
#define __KNOWROB_QUERY_H__

namespace knowrob {
    /**
     * A query that is represented by one or more predicates.
     * The query is evaluated by determining the truth of these predicates.
     * In case variables appear in arguments of the predicate, the answer
     * to the query contains possible instantiations of these variables.
     */
    class Query {
    public:
        Query(const std::string &queryString);
        Query(const Predicate &queryPredicate);
        ~Query();

        /** Parse a query string into a sequence of predicates.
         *
         * @param queryString the query string
         */
        virtual void parseQueryString(const std::string &queryString) = 0;

        /** 
         */
        const std::array<Predicate>& conjuncts() const { return predicates_; }
        const std::array<Predicate>& disjuncts() const { return predicates_; }

        /** Converts this query to a Prolog query string.
         *
         * @return the Prolog query string.
         * @todo rather do this in Prolog-specific code?
         */
        std::string toPrologQueryString();

        bool isConjunctive();
        bool isAtomic();

    protected:
        std::array<Predicate> predicates_;
    };
}

#endif //__KNOWROB_QUERY_H__
