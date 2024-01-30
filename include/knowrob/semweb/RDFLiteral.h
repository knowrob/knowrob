//
// Created by daniel on 07.04.23.
//

#ifndef KNOWROB_FRAMED_RDF_LITERAL_H
#define KNOWROB_FRAMED_RDF_LITERAL_H

#include "knowrob/formulas/Predicate.h"
#include "knowrob/semweb/StatementData.h"
#include "knowrob/terms/Constant.h"
#include "knowrob/formulas/Literal.h"
#include "knowrob/queries/QueryContext.h"

namespace knowrob {
    /**
     * A triple expression where subject, predicate and object are
     * represented as a term, and an additional unary operator can be applied to the object.
     */
    class RDFLiteral : public Literal {
    public:
        /**
         * Unary operators that can be applied on terms.
         */
        enum OperatorType { EQ, LT, GT, LEQ, GEQ };

		/**
		 * Copy char data of StatementData object into Term data structures.
		 * @param tripleData input data, can be deleted afterwards.
		 * @param isNegated a value of true refers to the statement being false.
		 */
        explicit RDFLiteral(const StatementData &tripleData, bool isNegated=false);

		/**
		 * @param predicate a predicate with two arguments.
		 * @param isNegated a value of true refers to the statement being false.
		 * @param selector a selector for the graph, agent, begin, end and confidence.
		 */
		RDFLiteral(const PredicatePtr &predicate, bool isNegated, const GraphSelector &selector);

        /**
         * Substitution constructor.
         * @other a literal.
         * @sub a mapping from terms to variables.
         */
        RDFLiteral(const RDFLiteral &other, const Substitution &sub);

        RDFLiteral(const TermPtr &s,
                   const TermPtr &p,
                   const TermPtr &o,
                   bool isNegated,
                   const GraphSelector &selector);

        /**
         * @return the subject term of this expression.
         */
        std::shared_ptr<Term> subjectTerm() const;

        /**
         * @return the property term of this expression.
         */
        std::shared_ptr<Term> propertyTerm() const;

        /**
         * @return the object term of this expression.
         */
        std::shared_ptr<Term> objectTerm() const;

        /**
         * @return the graph term of this expression.
         */
        std::shared_ptr<Term> graphTerm() const;

        void setGraphName(const std::string_view &graphName);

        /**
         * @return the agent term of this expression.
         */
        std::shared_ptr<Term> agentTerm() const;

        /**
         * @return the begin term of this expression.
         */
        std::shared_ptr<Term> beginTerm() const;

        /**
         * @return the end term of this expression.
         */
        std::shared_ptr<Term> endTerm() const;

        /**
         * @return the confidence term of this expression.
         */
        std::shared_ptr<Term> confidenceTerm() const;

        /**
         * @return the operator for the object of the triple.
         */
        OperatorType objectOperator() const;

        std::optional<TemporalOperator> temporalOperator() const { return temporalOperator_; };

        std::optional<EpistemicOperator> epistemicOperator() const { return epistemicOperator_; };

        void setObjectOperator(OperatorType objectOperator) { objectOperator_ = objectOperator; }

        uint32_t numVariables() const override;

        StatementData toStatementData() const;

    protected:
        std::shared_ptr<Term> subjectTerm_;
        std::shared_ptr<Term> propertyTerm_;
        std::shared_ptr<Term> objectTerm_;
        OperatorType objectOperator_;

		// below are treated as optional
        std::shared_ptr<Term> graphTerm_;
        std::shared_ptr<Term> agentTerm_;
        std::shared_ptr<Term> beginTerm_;
        std::shared_ptr<Term> endTerm_;
        std::shared_ptr<Term> confidenceTerm_;
        std::optional<TemporalOperator> temporalOperator_;
        std::optional<EpistemicOperator> epistemicOperator_;

        static std::shared_ptr<Term> getGraphTerm(const std::string_view &graphName);

        static std::shared_ptr<Predicate> getRDFPredicate(const TermPtr &s, const TermPtr &p, const TermPtr &o);
        static std::shared_ptr<Predicate> getRDFPredicate(const StatementData &data);
        static std::shared_ptr<Predicate> getRDFPredicate(const PredicatePtr &predicate);
    };
    using RDFLiteralPtr = std::shared_ptr<RDFLiteral>;

} // knowrob

#endif //KNOWROB_FRAMED_RDF_LITERAL_H
