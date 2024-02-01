/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_FORMULA_QUERY_H_
#define KNOWROB_FORMULA_QUERY_H_

#include <knowrob/queries/Query.h>

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
		FormulaQuery(const FormulaPtr &formula, const QueryContextPtr &ctx);

		/**
		 * Replaces variables in the query with terms based on a mapping provided in the argument.
		 * @sub a mapping from variables to terms.
		 * @return the new query created.
		 */
		std::shared_ptr<FormulaQuery> applySubstitution(const Substitution &sub) const;

		// Override Query
		const FormulaPtr &formula() const override { return formula_; }

		// Override Query
		std::ostream &print(std::ostream &os) const override;

		// Override Query
		QueryType type() const override { return QueryType::FORMULA; }

	protected:
		const std::shared_ptr<Formula> formula_;
	};
}

#endif //KNOWROB_FORMULA_QUERY_H_
