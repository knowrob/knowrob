/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_FRAMED_TRIPLE_PATTERN_H
#define KNOWROB_FRAMED_TRIPLE_PATTERN_H

#include "knowrob/formulas/Predicate.h"
#include "knowrob/triples/FramedTriple.h"
#include "knowrob/triples/TripleContainer.h"
#include "knowrob/formulas/FirstOrderLiteral.h"
#include "knowrob/queries/QueryContext.h"
#include "knowrob/terms/groundable.h"
#include "knowrob/terms/Numeric.h"
#include "knowrob/terms/IRIAtom.h"
#include "knowrob/terms/Blank.h"

namespace knowrob {
	/**
	 * A triple expression where subject, predicate and object are
	 * represented as a term, and an additional unary operator can be applied to the object.
	 */
	class FramedTriplePattern : public FirstOrderLiteral {
	public:
		/**
		 * Unary operators that can be applied on terms.
		 */
		enum OperatorType {
			EQ, LT, GT, LEQ, GEQ
		};

		/**
		 * Copy char data of StatementData object into Term data structures.
		 * @param tripleData input data, can be deleted afterwards.
		 * @param isNegated a value of true refers to the statement being false.
		 */
		explicit FramedTriplePattern(const FramedTriple &triple, bool isNegated = false);

		/**
		 * @param predicate a predicate with two arguments.
		 * @param isNegated a value of true refers to the statement being false.
		 * @param selector a selector for the graph, agent, begin, end and confidence.
		 */
		explicit FramedTriplePattern(const PredicatePtr &pred, bool isNegated = false);

		/**
		 * @param s the subject term.
		 * @param p the property term.
		 * @param o the object term.
		 * @param isNegated a value of true refers to the statement being false.
		 */
		FramedTriplePattern(const TermPtr &s, const TermPtr &p, const TermPtr &o, bool isNegated = false);

		/**
		 * @param frame the graph, agent, begin, end and confidence.
		 */
		void setTripleFrame(const GraphSelector &frame);

		/**
		 * @return the subject term of this expression.
		 */
		auto &subjectTerm() const { return subjectTerm_; }

		/**
		 * @return the property term of this expression.
		 */
		auto &propertyTerm() const { return propertyTerm_; }

		/**
		 * @return the object term of this expression.
		 */
		auto &objectTerm() const { return objectTerm_; }

		/**
		 * @return the graph term of this expression.
		 */
		auto &graphTerm() const { return graphTerm_; }

		/**
		 * Set the graph term of this expression.
		 * @param graphTerm the graph term.
		 */
		void setGraphTerm(const groundable<Atom> &graphTerm) { graphTerm_ = graphTerm; }

		/**
		 * Set the graph term of this expression.
		 * @param graphName the name of the graph.
		 */
		void setGraphName(const std::string_view &graphName) { graphTerm_ = getGraphTerm(graphName); }

		/**
		 * @return the agent term of this expression.
		 */
		auto &perspectiveTerm() const { return perspectiveTerm_; }

		/**
		 * Set the agent term of this expression.
		 * @param agentTerm the agent term.
		 */
		void setPerspectiveTerm(const groundable<Atom> &perspectiveTerm) { perspectiveTerm_ = perspectiveTerm; }

		/**
		 * @return the begin term of this expression.
		 */
		auto &beginTerm() const { return beginTerm_; }

		/**
		 * Set the begin term of this expression.
		 * @param beginTerm the begin term.
		 */
		void setBeginTerm(const groundable<Double> &beginTerm) { beginTerm_ = beginTerm; }

		/**
		 * @return the end term of this expression.
		 */
		auto &endTerm() const { return endTerm_; }

		/**
		 * Set the end term of this expression.
		 * @param endTerm the end term.
		 */
		void setEndTerm(const groundable<Double> &endTerm) { endTerm_ = endTerm; }

		/**
		 * @return the confidence term of this expression.
		 */
		auto &confidenceTerm() const { return confidenceTerm_; }

		/**
		 * Set the confidence term of this expression.
		 * @param confidenceTerm the confidence term.
		 */
		void setConfidenceTerm(const groundable<Double> &confidenceTerm) { confidenceTerm_ = confidenceTerm; }

		/**
		 * @return the operator for the object of the triple.
		 */
		auto objectOperator() const { return objectOperator_; }

		/**
		 * Set the operator for the object of the triple.
		 * @param objectOperator the operator.
		 */
		void setObjectOperator(OperatorType objectOperator) { objectOperator_ = objectOperator; }

		/**
		 * @return the isOccasional term of this expression.
		 */
		auto &isOccasionalTerm() const { return isOccasional_; }

		/**
		 * Set the isOccasional term of this expression.
		 * @param isOccasional the isOccasional term.
		 */
		void setIsOccasionalTerm(const groundable<Numeric> &isOccasional) { isOccasional_ = isOccasional; }

		/**
		 * @return the isUncertain term of this expression.
		 */
		auto &isUncertainTerm() const { return isUncertain_; }

		/**
		 * Set the isUncertain term of this expression.
		 * @param isUncertain the isUncertain term.
		 */
		void setIsUncertainTerm(const groundable<Numeric> &isUncertain) { isUncertain_ = isUncertain; }

		/**
		 * @return true if this expression is optional.
		 */
		bool isOptional() const { return isOptional_; }

		/**
		 * Set this expression to be optional.
		 * @param isOptional true if this expression is optional.
		 */
		void setIsOptional(bool isOptional) { isOptional_ = isOptional; }

		/**
		 * @return the number of variables in this expression.
		 */
		uint32_t numVariables() const override;

		/**
		 * @return the variables in this expression.
		 */
		std::vector<VariablePtr> getVariables() const;

		/**
		 * Map the instantiation of this expression into a triple.
		 * @param triple the triple to be instantiated.
		 * @param bindings the substitution to be applied.
		 * @return true if the instantiation was successful.
		 */
		bool instantiateInto(FramedTriple &triple,
							 const std::shared_ptr<const Bindings> &bindings = Bindings::emptyBindings()) const;

	protected:
		TermPtr subjectTerm_;
		TermPtr propertyTerm_;
		TermPtr objectTerm_;
		OperatorType objectOperator_;
		bool isOptional_;

		// below are treated as optional
		groundable<Atom> graphTerm_;
		groundable<Atom> perspectiveTerm_;
		groundable<Double> beginTerm_;
		groundable<Double> endTerm_;
		groundable<Double> confidenceTerm_;
		groundable<Numeric> isOccasional_;
		groundable<Numeric> isUncertain_;

		static std::shared_ptr<Atom> getGraphTerm(const std::string_view &graphName);

		static std::shared_ptr<Predicate> getRDFPredicate(const TermPtr &s, const TermPtr &p, const TermPtr &o);

		static std::shared_ptr<Predicate> getRDFPredicate(const FramedTriple &data);

		static std::shared_ptr<Predicate> getRDFPredicate(const PredicatePtr &predicate);
	};

	/**
	 * A shared pointer to a framed triple pattern.
	 */
	using FramedTriplePatternPtr = std::shared_ptr<FramedTriplePattern>;

	/**
	 * Apply a substitution to a framed triple pattern.
	 * @param pat the framed triple pattern.
	 * @param bindings the substitution.
	 * @return the framed triple pattern with the substitution applied.
	 */
	FramedTriplePatternPtr applyBindings(const FramedTriplePatternPtr &pat, const Bindings &bindings);

	/**
	 * A container that maps a vector of framed triple patterns into a vector of framed triples.
	 */
	class TriplePatternContainer : public semweb::MutableTripleContainer {
	public:
		TriplePatternContainer() = default;

		~TriplePatternContainer();

		TriplePatternContainer(const TriplePatternContainer &other) = delete;

		/**
		 * @param triple a triple query.
		 */
		void push_back(const FramedTriplePatternPtr &q);

		// Override TripleContainer
		ConstGenerator cgenerator() const override;

		// Override MutableTripleContainer
		MutableGenerator generator() override;

	protected:
		std::vector<FramedTriplePtr*> data_;
		std::vector<FramedTriplePatternPtr> statements_;
	};
} // knowrob

#endif //KNOWROB_FRAMED_TRIPLE_PATTERN_H
