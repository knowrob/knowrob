/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERY_H_
#define KNOWROB_QUERY_H_

#include <memory>
#include <ostream>
#include <optional>
#include <utility>
#include "knowrob/formulas/Predicate.h"
#include <knowrob/formulas/Formula.h>
#include "knowrob/modalities/TimeInterval.h"
#include "knowrob/modalities/ConfidenceInterval.h"
#include "QueryContext.h"

namespace knowrob {
	enum class QueryType : uint8_t  {
		CONJUNCTIVE = 0,
		FORMULA
	};

	/**
	 * A query represented by a propositional formula.
	 * @deprecated use Formula instead
	 */
	class Query {
	public:
		/**
		 * @formula the formula associated to this query.
		 */
		explicit Query(QueryContextPtr ctx) : ctx_(std::move(ctx)) {}

		virtual QueryType type() const = 0;

		static int defaultFlags();

		int flags() const { return ctx_->queryFlags_; }

		auto &ctx() const { return ctx_; }

		virtual std::ostream &print(std::ostream &os) const = 0;

		/**
		 * @return the formula associated to this query.
		 */
		virtual const FormulaPtr &formula() const = 0;

	protected:
		QueryContextPtr ctx_;
	};

	// alias
	using QueryPtr = std::shared_ptr<const Query>;
}

namespace std {
	std::ostream &operator<<(std::ostream &os, const knowrob::Query &q);
}

#endif //KNOWROB_QUERY_H_
