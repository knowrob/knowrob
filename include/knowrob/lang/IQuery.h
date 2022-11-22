/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_IQUERY_H__
#define __KNOWROB_IQUERY_H__

namespace knowrob {
    /**
     * A query that is represented by one or more predicates.
     * The query is evaluated by determining the truth of these predicates.
     * In case variables appear in arguments of the predicate, the answer
     * to the query contains possible instantiations of these variables.
     */
    class IQuery {
    public:
        IQuery(const std::string &queryString);
        IQuery(const Predicate &queryPredicate);
        ~IQuery();

        /** Parse a query string into a sequence of predicates.
         *
         * @param queryString the query string
         */
        virtual void parseQueryString(const std::string &queryString) = 0;

        /** Get the toplevel predicates that make this query.
         *
         * Predicates may be nested in which case they contain references
         * to other predicates. For instance, for control structures such
         * as collecting different solutions.
         *
         * @param predicate the predicate in question
         * @return true if the reasoner can determine the truth of given predicate.
         */
        const std::array<Predicate>& predicates() const { return predicates_; }

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

#endif //__KNOWROB_IQUERY_H__
