//
// Created by daniel on 07.04.23.
//

#ifndef KNOWROB_SEMWEB_TRIPLE_EXPRESSION_H
#define KNOWROB_SEMWEB_TRIPLE_EXPRESSION_H

#include "knowrob/formulas/Predicate.h"
#include "knowrob/semweb/StatementData.h"
#include "knowrob/terms/Constant.h"

namespace knowrob::semweb {
    /**
     * A triple expression where subject, predicate and object are
     * represented as a term, and an additional unary operator can be applied to the object.
     */
    class FramedLiteral {
    public:
        /**
         * Unary operators that can be applied on terms.
         */
        enum OperatorType { EQ, LT, GT, LEQ, GEQ };

        FramedLiteral(const TermPtr &subjectTerm,
                      const TermPtr &propertyTerm,
                      const TermPtr &objectTerm,
                      OperatorType objectOperator=EQ,
                      const std::string_view &graphName="*");

        explicit FramedLiteral(const PredicatePtr &triplePredicate,
                               const std::string_view &graphName="*");

        explicit FramedLiteral(const StatementData &tripleData);

        /**
         * @return true if the expression has no variables.
         */
        bool isGround() const;

        /**
         * @return the subject term of this expression.
         */
        auto& subjectTerm() const { return subjectTerm_; }

        /**
         * @return the property term of this expression.
         */
        auto& propertyTerm() const { return propertyTerm_; }

        /**
         * @return the object term of this expression.
         */
        auto& objectTerm() const { return objectTerm_; }

        /**
         * @return the graph term of this expression.
         */
        auto& graphTerm() const { return graphTerm_; }

        /**
         * @return the agent term of this expression.
         */
        auto& agentTerm() const { return agentTerm_; }

        /**
         * @return the begin term of this expression.
         */
        auto& beginTerm() const { return beginTerm_; }

        /**
         * @return the end term of this expression.
         */
        auto& endTerm() const { return endTerm_; }

        /**
         * @return the confidence term of this expression.
         */
        auto& confidenceTerm() const { return confidenceTerm_; }

        /**
         * @return the operator for the object of the triple.
         */
        auto objectOperator() const { return objectOperator_; }

        /**
         * @return the operator for the confidence of the triple.
         */
        auto confidenceOperator() const { return confidenceOperator_; }

        /**
         * @return the operator for the begin of the triple.
         */
        auto beginOperator() const { return beginOperator_; }

        /**
         * @return the operator for the end of the triple.
         */
        auto endOperator() const { return endOperator_; }

        /**
         * @param limit the minimum confidence of triples matching this expression
         */
        void setMinConfidence(double limit);

        /**
         * @param limit the maximum confidence of triples matching this expression
         */
        void setMaxConfidence(double limit);

        /**
         * @param limit the minimum begin time of triples matching this expression
         */
        void setMinBegin(double limit);

        /**
         * @param limit the maximum begin time of triples matching this expression
         */
        void setMaxBegin(double limit);

        /**
         * @param limit the minimum end time of triples matching this expression
         */
        void setMinEnd(double limit);

        /**
         * @param limit the maximum end time of triples matching this expression
         */
        void setMaxEnd(double limit);

        /**
         * @param beginOperator the operator used for the begin time of triples.
         */
        void setBeginOperator(OperatorType beginOperator) { beginOperator_ = beginOperator; }

        /**
         * @param beginOperator the operator used for the end time of triples.
         */
        void setEndOperator(OperatorType endOperator) { endOperator_ = endOperator; }

        /**
         * @param beginTerm a time term.
         */
        void setBeginTerm(const TermPtr &beginTerm) { beginTerm_ = beginTerm; }

        /**
         * @param endTerm a time term.
         */
        void setEndTerm(const TermPtr &endTerm) { endTerm_ = endTerm; }

        void setAgentTerm(const std::string &agentTerm)
        { agentTerm_ = std::make_shared<StringTerm>(agentTerm); }

    protected:
        TermPtr subjectTerm_;
        TermPtr propertyTerm_;
        TermPtr objectTerm_;
        TermPtr graphTerm_;
        TermPtr agentTerm_;
        TermPtr beginTerm_;
        TermPtr endTerm_;
        TermPtr confidenceTerm_;
        OperatorType objectOperator_;
        OperatorType beginOperator_;
        OperatorType endOperator_;
        OperatorType confidenceOperator_;
    };

} // knowrob

#endif //KNOWROB_SEMWEB_TRIPLE_EXPRESSION_H
