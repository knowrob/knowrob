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

		explicit AnswerYes(SubstitutionPtr substitution);

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
		 * Map a variable to a term.
		 * @var a variable.
		 * @term a term.
		 */
		void set(const std::shared_ptr<Variable> &var, const TermPtr &term) { substitution_->set(var, term); }

		/**
		 * @param var a variable.
		 * @return true if the variable is mapped to a term.
		 */
		bool hasGrounding(const Variable &var) const { return substitution_->contains(var.name()); }

		/**
		 * The answer is framed in the context of a graph selector which determines
		 * the set of graphs in which the answer is valid.
		 * This can be used to e.g. address graphs that represent the world state from the
		 * perspective of a specific agent, or a specific point in time.
		 * @return a graph selector.
		 */
		auto &frame() { return frame_; }

		/**
		 * Assign a graph selector to this answer.
		 * @param frame a graph selector.
		 */
		void setFrame(const std::shared_ptr<GraphSelector> &frame) { frame_ = frame; }

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

		// override Answer
		bool isUncertain() const override;

		// override Answer
		void setIsUncertain(std::optional<double> confidence) override;

	protected:
		std::vector<FramedPredicate> positiveGroundings_;
		std::vector<FramedPredicate> negativeGroundings_;
		SubstitutionPtr substitution_;
		std::shared_ptr<GraphSelector> frame_;
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
