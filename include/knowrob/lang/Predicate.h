/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_PREDICATE_H__
#define __KNOWROB_PREDICATE_H__

#include <string>

namespace knowrob {
    /**
     * A predicate in the knowledge base defined by its functor and arity.
     */
    class Predicate {
    public:
        Predicate(const PredicateIndicator &indicator)
            : indicator_(indicator) {};

        Predicate(const std::string &functor, unsigned int arity)
            : indicator_(functor, arity) {};

        /** Get the functor of this predicate.
         *
         * @return functor name
         */
        const std::string& functor() const { return indicator_.functor(); }

        /** Get the arity of this predicate.
         *
         * @return arity of predicate
         */
        unsigned int arity() const { return indicator_.arity(); }

    private:
        PredicateIndicator indicator_;
    };
}

#endif //__KNOWROB_PREDICATE_H__