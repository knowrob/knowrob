/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_PREDICATE_INDICATOR_H__
#define __KNOWROB_PREDICATE_INDICATOR_H__

#include <string>

namespace knowrob {
    /**
     * A predicate in the knowledge base defined by its functor and arity.
     */
    class PredicateIndicator {
    public:
        PredicateIndicator(const std::string &functor, unsigned int arity)
            : functor_(functor), arity_(arity) {};

        /** Get the functor of this predicate.
         *
         * @return functor name
         */
        const std::string& functor() const { return functor_; }

        /** Get the arity of this predicate.
         *
         * @return arity of predicate
         */
        unsigned int arity() const { return arity_; }

    private:
        std::string functor_;
        unsigned int arity_;
    };
}

#endif //__KNOWROB_PREDICATE_INDICATOR_H__
