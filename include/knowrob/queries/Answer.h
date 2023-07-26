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
#include "knowrob/modalities/ModalFrame.h"

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
		 * @return the modal frame of this proposition.
		 */
		const ModalFrame& modalFrame() const { return modality_; }

		/**
		 * @param modality the modal frame of this proposition.
		 */
		void setModalFrame(const ModalFrame &modality) { modality_ = modality; }

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
		ModalFrame modality_;

		bool combineModalFrame(const std::shared_ptr<const Answer> &other);

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
