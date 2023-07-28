/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERY_H_
#define KNOWROB_QUERY_H_

#include <memory>
#include <ostream>
#include <optional>
#include "knowrob/formulas/Predicate.h"
#include <knowrob/formulas/Formula.h>
#include "knowrob/modalities/TimeInterval.h"
#include "knowrob/modalities/ConfidenceInterval.h"
#include "knowrob/modalities/ModalityFrame.h"

namespace knowrob {
	/**
	 * A query represented by a propositional formula.
	 * @deprecated use Formula instead
	 */
	class Query {
	public:
		/**
		 * @formula the formula associated to this query.
		 */
        Query(const FormulaPtr &formula, int flags);

		/**
		 * @return the formula associated to this query.
		 */
		const FormulaPtr& formula() const { return formula_; }

        int flags() const { return flags_; }
		
		/**
		 * Replaces variables in the query with terms based on a mapping provided in the argument.
		 * @sub a mapping from variables to terms.
		 * @return the new query created.
		 */
		std::shared_ptr<Query> applySubstitution(const Substitution &sub) const;

	protected:
		const std::shared_ptr<Formula> formula_;
		const int flags_;
	};
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::Query& q);
}

#endif //KNOWROB_QUERY_H_
