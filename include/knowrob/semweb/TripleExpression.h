//
// Created by daniel on 07.04.23.
//

#ifndef KNOWROB_SEMWEB_TRIPLE_EXPRESSION_H
#define KNOWROB_SEMWEB_TRIPLE_EXPRESSION_H

#include "knowrob/formulas/Predicate.h"
#include "TripleData.h"

namespace knowrob::semweb {
    /**
     * A triple expression where subject, predicate and object are
     * represented as a term, and an additional unary operator can be applied to the object.
     */
    class TripleExpression {
    public:
        /**
         * Unary operators that can be applied on terms.
         */
        enum OperatorType { EQ, LT, GT, LEQ, GEQ };

        TripleExpression(const TermPtr &subjectTerm,
                         const TermPtr &propertyTerm,
                         const TermPtr &objectTerm,
                         OperatorType objectOperator=EQ,
                         const std::string_view &graphName="*");

        explicit TripleExpression(const PredicatePtr &triplePredicate,
                                  const std::string_view &graphName="*");

        explicit TripleExpression(const TripleData &tripleData);

        bool isGround() const;

        auto& subjectTerm() const { return subjectTerm_; }

        auto& propertyTerm() const { return propertyTerm_; }

        auto& objectTerm() const { return objectTerm_; }

        auto& graphTerm() const { return graphTerm_; }

        auto& beginTerm() const { return beginTerm_; }

        auto& endTerm() const { return endTerm_; }

        auto& confidenceTerm() const { return confidenceTerm_; }

        auto objectOperator() const { return objectOperator_; }

        auto confidenceOperator() const { return confidenceOperator_; }

        auto beginOperator() const { return beginOperator_; }

        auto endOperator() const { return endOperator_; }

        void setMinConfidence(double limit);

        void setMaxConfidence(double limit);

        void setMinBegin(double limit);

        void setMaxBegin(double limit);

        void setMinEnd(double limit);

        void setMaxEnd(double limit);

        void setBeginOperator(OperatorType beginOperator) { beginOperator_ = beginOperator; }

        void setEndOperator(OperatorType endOperator) { endOperator_ = endOperator; }

        void setBeginTerm(const TermPtr &beginTerm) { beginTerm_ = beginTerm; }

        void setEndTerm(const TermPtr &endTerm) { endTerm_ = endTerm; }

    protected:
        TermPtr subjectTerm_;
        TermPtr propertyTerm_;
        TermPtr objectTerm_;
        TermPtr graphTerm_;
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
