//
// Created by daniel on 07.04.23.
//

#ifndef KNOWROB_FRAMED_RDF_LITERAL_H
#define KNOWROB_FRAMED_RDF_LITERAL_H

#include "knowrob/formulas/Predicate.h"
#include "knowrob/semweb/StatementData.h"
#include "knowrob/terms/Constant.h"
#include "knowrob/formulas/Literal.h"
#include "knowrob/modalities/ModalityFrame.h"

namespace knowrob {
    /**
     * A triple expression where subject, predicate and object are
     * represented as a term, and an additional unary operator can be applied to the object.
     */
    class FramedRDFLiteral {
    public:
        /**
         * Unary operators that can be applied on terms.
         */
        enum OperatorType { EQ, LT, GT, LEQ, GEQ };

        explicit FramedRDFLiteral(const LiteralPtr &literal, const ModalityFrame &modalityFrame=ModalityFrame());

        explicit FramedRDFLiteral(const StatementData &tripleData);

        FramedRDFLiteral(const TermPtr &subjectTerm,
                         const TermPtr &propertyTerm,
                         const TermPtr &objectTerm,
                         OperatorType objectOperator=EQ,
                         const std::string_view &graphName="*");

/*
        explicit FramedRDFLiteral(const PredicatePtr &triplePredicate,
                               const std::string_view &graphName="*");
*/

        const ModalityFrame& modalityFrame() const { return modalityFrame_; }

        /**
         * @return true if the expression has no variables.
         */
        bool isGround() const;

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

        /**
         * @return the operator for the confidence of the triple.
         */
        OperatorType confidenceOperator() const;

        /**
         * @param limit the minimum confidence of triples matching this expression
         */
        void setMinConfidence(double limit);

        /**
         * @param limit the maximum confidence of triples matching this expression
         */
        void setMaxConfidence(double limit);

        /**
         * @param beginTerm a time term.
         */
        void setBeginTerm(const TermPtr &beginTerm);

        /**
         * @param endTerm a time term.
         */
        void setEndTerm(const TermPtr &endTerm);

        void setAgentTerm(const std::string &agentTerm);

        StatementData toStatementData() const;

    protected:
        ModalityFrame modalityFrame_;
        LiteralPtr  literal_;

        std::shared_ptr<Term> subjectTerm_;
        std::shared_ptr<Term> propertyTerm_;
        std::shared_ptr<Term> objectTerm_;
        std::shared_ptr<Term> graphTerm_;
        std::shared_ptr<Term> agentTerm_;
        std::shared_ptr<Term> beginTerm_;
        std::shared_ptr<Term> endTerm_;
        std::shared_ptr<Term> confidenceTerm_;
        OperatorType objectOperator_;
        OperatorType confidenceOperator_;
    };
    using FramedRDFLiteralPtr = std::shared_ptr<FramedRDFLiteral>;

} // knowrob

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::FramedRDFLiteral& l);
}

#endif //KNOWROB_FRAMED_RDF_LITERAL_H
