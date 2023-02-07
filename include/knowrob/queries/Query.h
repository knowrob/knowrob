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
#include <knowrob/terms.h>
#include <knowrob/formulas/Formula.h>
#include "knowrob/scope/TimeInterval.h"
#include "knowrob/scope/ConfidenceInterval.h"

namespace knowrob {
	/**
	 * A query represented by a propositional formula.
	 */
	class Query {
	public:
		/**
		 * @formula the formula associated to this query.
		 */
		explicit Query(const FormulaPtr &formula);
		
		/**
		 * @predicate the predicate that is queried.
		 */
		explicit Query(const std::shared_ptr<Predicate> &predicate);

		/**
		 * @return the formula associated to this query.
		 */
		const FormulaPtr& formula() const { return formula_; }
		
		/**
		 * Replaces variables in the query with terms based on a mapping provided in the argument.
		 * @sub a mapping from variables to terms.
		 * @return the new query created.
		 */
		std::shared_ptr<Query> applySubstitution(const Substitution &sub) const;

		/**
		 * Assigns a time interval to this query indicating that solutions
		 * should only be generated that are valid within this interval.
		 * @param timeInterval the time interval of this query.
		 */
		void setTimeInterval(const std::shared_ptr<TimeInterval> &timeInterval);

		/**
		 * Assigns a confidenceInterval interval to this query indicating that solutions
		 * should only be generated that are valid within this interval.
		 * @param confidence the confidenceInterval interval of this query.
		 */
		void setConfidenceInterval(const std::shared_ptr<ConfidenceInterval> &confidenceInterval);

		/**
		 * @return an optional time interval of this query.
		 */
		const std::optional<const TimeInterval*>& timeInterval() const { return o_timeInterval_; }

		/**
		 * @return an optional confidenceInterval interval of this query.
		 */
		const std::optional<const ConfidenceInterval*>& confidenceInterval() const { return o_confidenceInterval_; }

	protected:
		const std::shared_ptr<Formula> formula_;
		std::shared_ptr<TimeInterval> timeInterval_;
		std::shared_ptr<ConfidenceInterval> confidenceInterval_;
		std::optional<const TimeInterval*> o_timeInterval_;
		std::optional<const ConfidenceInterval*> o_confidenceInterval_;
		friend class QueryInstance;
	};
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::Query& q);
}

#endif //KNOWROB_QUERY_H_
