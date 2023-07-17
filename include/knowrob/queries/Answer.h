/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERY_RESULT_H_
#define KNOWROB_QUERY_RESULT_H_

#include <memory>
#include <list>
#include <optional>
#include <ostream>
#include "knowrob/terms/Term.h"
#include "knowrob/terms/PredicateInstance.h"
#include "knowrob/modalities/TimeInterval.h"
#include "knowrob/modalities/ConfidenceValue.h"

namespace knowrob {
	/**
	 * The result of query evaluation.
	 * A result indicates that the evaluation succeeded, i.e.,
	 * that a reasoner was able to find an instance of the query that is true.
	 */
	class Answer {
	public:
		Answer();

		/**
		 * Copy another result.
		 * Modification of the constructed result won't affect the copied one.
		 * @param other another query result.
		 */
		Answer(const Answer &other);

		/**
		 * @return a positive result without additional constraints.
		 */
		static const std::shared_ptr<const Answer>& emptyResult();

		/**
		 * Adds to this result a substitution of a variable with a term.
		 * @param var a variable
		 * @param term a term
		 */
		void substitute(const Variable &var, const TermPtr &term);

		/**
		 * @param var a variable.
		 * @return true is this solution substitutes the variable
		 */
		bool hasSubstitution(const Variable &var);

		/**
		 * @return a mapping from variables to terms.
		 */
		const SubstitutionPtr& substitution() const { return substitution_; }

		/**
		 * @param reasonerModule the reasoner module the inferred the instance
		 * @param instance an instance of a query predicate
		 */
		void addPredicate(const std::shared_ptr<StringTerm> &reasonerModule,
						  const std::shared_ptr<Predicate> &predicate);

		/**
		 * A list of all query predicates that were instantiated to reach
		 * this solution. Only predicates that appear in the user query are
		 * included in this list.
		 * @return instantiated predicates.
		 */
		const std::list<PredicateInstance>& predicates() const { return predicates_; }

		/**
		 * Assigns a time interval to this solution indicating that the solution
		 * is only valid in a restricted time frame.
		 * @param timeInterval the time interval of the result being valid.
		 */
		void setTimeInterval(const std::shared_ptr<TimeInterval> &timeInterval);

		/**
		 * Assigns a confidenceInterval value to this solution.
		 * @param confidence a confidenceInterval value of the result being valid.
		 */
		void setConfidenceValue(const std::shared_ptr<ConfidenceValue> &confidence);

		/**
		 * @return an optional time interval of the result being valid.
		 */
		const std::optional<const TimeInterval*>& timeInterval() const { return o_timeInterval_; }

		/**
		 * @return an optional confidenceInterval value of the result being valid.
		 */
		const std::optional<const ConfidenceValue*>& confidence() const { return o_confidence_; }

		/**
		 * Merge another query result into this one.
		 * A merge failure is indicated by the return value, e.g. in case
		 * both substitutions cannot be unified false is returned.
		 * @param other another query result.
		 * @param changes used to make the merge operation reversible, can be null.
		 * @return false if merge is not possible.
		 */
		bool combine(const std::shared_ptr<const Answer> &other, Reversible *changes=nullptr);

	protected:
		SubstitutionPtr substitution_;
		std::list<PredicateInstance> predicates_;
		std::shared_ptr<TimeInterval> timeInterval_;
		std::shared_ptr<ConfidenceValue> confidence_;
		std::optional<const TimeInterval*> o_timeInterval_;
		std::optional<const ConfidenceValue*> o_confidence_;

		bool combineConfidence(const std::shared_ptr<ConfidenceValue> &otherConfidence);

		bool combineTimeInterval(const std::shared_ptr<TimeInterval> &otherTimeInterval,
								 Reversible *changes= nullptr);

		friend class AllocatedQuery;
	};
	// alias
	using AnswerPtr = std::shared_ptr<const Answer>;
	using AnswerMap = std::map<uint32_t, std::list<AnswerPtr>>;

    class QueryResultHandler {
    public:
        virtual bool pushQueryResult(const AnswerPtr &solution) = 0;
    };

}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::Answer& solution);
}

#endif //KNOWROB_QUERY_RESULT_H_
