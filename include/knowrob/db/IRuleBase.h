/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_RULE_BASE_H__
#define __KNOWROB_RULE_BASE_H__

#include <set>

#include "knowrob/db/IDataSource.h"
#include "knowrob/lang/PredicateIndicator.h"

namespace knowrob {
    /**
     * A collection of rules used to perform deductive reasoning.
     */
    class IRuleBase<RuleType> : public IDataSource {
    public:
        virtual ~IRuleBase(){}

        /** Get list of IDB predicates in this rule base.
         *
         * @return list of predicate indicators
         */
        const std::set<PredicateIndicator>& predicateIndicators() const
            { return predicateIndicators_; }

        /** Get whether the rule base has particular IDB predicate.
         *
         * @param indicator a predicate indicator
         * @return true if the indicator is the one of an IDB predicate of this rule base
         */
        bool containsPredicate(const PredicateIndicator &indicator) const
            { return predicateIndicators_.find(indicator) != predicateIndicators_.end(); }

    protected:
        std::set<PredicateIndicator> predicateIndicators_;
    };
}

#endif //__KNOWROB_RULE_BASE_H__
