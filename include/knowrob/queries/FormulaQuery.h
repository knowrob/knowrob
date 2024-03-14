/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_FORMULA_QUERY_H_
#define KNOWROB_FORMULA_QUERY_H_

#include <knowrob/queries/Query.h>
#include <utility>

namespace knowrob {
	/**
	 * A query represented by a logical formula.
	 */
	class FormulaQuery : public Query {
	public:
		/**
		 * @param formula the formula to query.
		 * @param ctx the query context.
		 */
		FormulaQuery(FormulaPtr formula, const QueryContextPtr &ctx) : Query(ctx), formula_(std::move(formula)) {}

		// Override Query
		const FormulaPtr &formula() const { return formula_; }

	protected:
		const std::shared_ptr<Formula> formula_;

		// Override Query
		void write(std::ostream &os) const override { os << *formula_; }
	};
}

#endif //KNOWROB_FORMULA_QUERY_H_
