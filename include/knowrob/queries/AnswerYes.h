/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_POSITIVE_ANSWER_H
#define KNOWROB_POSITIVE_ANSWER_H

#include "Answer.h"
#include "knowrob/terms/PredicateInstance.h"
#include "knowrob/formulas/FirstOrderLiteral.h"
#include "knowrob/formulas/FramedPredicate.h"

namespace knowrob {
	/**
	 * A positive answer indicates that a querying component has evidence
	 * for the input query being true for some instances of the query.
	 */
	class AnswerYes : public Answer {
	public:
		/**
		 * Default constructor.
		 */
		AnswerYes();

		explicit AnswerYes(BindingsPtr substitution);

		/**
		 * Copy constructor.
		 * @param other another answer.
		 */
		AnswerYes(const AnswerYes &other);

		/**
		 * @param other another answer.
		 * @return true if this answer has more information than the other answer.
		 */
		bool isRicherThan(const AnswerYes &other) const;

		/**
		 * @return true if this answer is a generic "yes" answer without any groundings.
		 */
		bool isGenericYes() const;

		/**
		 * Each positive answer is associated with a substitution that maps
		 * variables to terms. This substitution can be used to instantiate
		 * the query.
		 * @return a mapping from variables to terms.
		 */
		auto &substitution() const { return substitution_; }

		/**
		 * @param var a variable.
		 * @return true if the variable is mapped to a term.
		 */
		bool hasGrounding(const Variable &var) const { return substitution_->contains(var.name()); }

		/**
		 * Add a grounded literal to the answer.
		 * Positive literals may not contain variables.t
		 * @param predicate a predicate.
		 * @param isNegated true if the negation of the predicate is grounded.
		 */
		bool addGrounding(const std::shared_ptr<Predicate> &predicate,
						  const GraphSelectorPtr &frame,
						  bool isNegated = false);

		/**
		 * Part of the answer is that certain literals that appear positive in the query
		 * are true. This is a list of such literals.
		 * @return a list of positive groundings.
		 */
		auto &positiveGroundings() const { return positiveGroundings_; }

		/**
		 * Part of the answer is that certain literals that appear negated in the query
		 * are not true. This is a list of such literals.
		 * @return a list of negative groundings.
		 */
		auto &negativeGroundings() const { return negativeGroundings_; }

		/**
		 * Merge this answer with another answer.
		 * @param other another answer.
		 * @param ignoreInconsistencies if true, inconsistencies are ignored.
		 * @return true if the merge was successful.
		 */
		bool mergeWith(const AnswerYes &other, bool ignoreInconsistencies = false);

		// override Token
		size_t hash() const override;

		// override Token
		std::ostream &write(std::ostream &os) const override;

		// override Answer
		std::string toHumanReadableString() const override;

		// override Answer
		bool isNegative() const override { return false; }

		// override Answer
		bool isPositive() const override { return true; }

	protected:
		std::vector<FramedPredicate> positiveGroundings_;
		std::vector<FramedPredicate> negativeGroundings_;
		BindingsPtr substitution_;
	};

	// alias
	using AnswerYesPtr = std::shared_ptr<const AnswerYes>;

	/**
	 * @return a positive result without additional constraints.
	 */
	const std::shared_ptr<const AnswerYes> &GenericYes();

	AnswerPtr mergePositiveAnswers(const AnswerYesPtr &a, const AnswerYesPtr &b, bool ignoreInconsistencies);

} // knowrob

#endif //KNOWROB_POSITIVE_ANSWER_H
