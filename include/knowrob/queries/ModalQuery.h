/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MODAL_QUERY_H_
#define KNOWROB_MODAL_QUERY_H_

#include <memory>
#include <ostream>
#include <optional>
#include "knowrob/formulas/Predicate.h"
#include "knowrob/formulas/Formula.h"
#include "knowrob/modalities/TimeInterval.h"
#include "knowrob/modalities/ConfidenceInterval.h"
#include "knowrob/queries/Query.h"

namespace knowrob {
	/**
	 * A query represented by a propositional formula.
	 * @deprecated use Formula instead
	 */
	class ModalQuery : public Query {
	public:
		/**
		 * @formula the formula associated to this query.
		 */
        ModalQuery(const FormulaPtr &formula, const QueryContextPtr &ctx);
		
		/**
		 * Replaces variables in the query with terms based on a mapping provided in the argument.
		 * @sub a mapping from variables to terms.
		 * @return the new query created.
		 */
		std::shared_ptr<ModalQuery> applySubstitution(const Substitution &sub) const;

        // Override Query
		const FormulaPtr& formula() const override { return formula_; }

        // Override Query
        std::ostream& print(std::ostream &os) const override;

        // Override Query
		QueryType type() const override { return QueryType::FORMULA; }

	protected:
		const std::shared_ptr<Formula> formula_;
	};
}

#endif //KNOWROB_MODAL_QUERY_H_
