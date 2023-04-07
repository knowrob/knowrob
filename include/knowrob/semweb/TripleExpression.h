//
// Created by daniel on 07.04.23.
//

#ifndef KNOWROB_SEMWEB_TRIPLE_EXPRESSION_H
#define KNOWROB_SEMWEB_TRIPLE_EXPRESSION_H

#include "knowrob/formulas/Predicate.h"

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
                         OperatorType objectOperator);

        explicit TripleExpression(const PredicatePtr &triplePredicate);

        auto& subjectTerm() const { return subjectTerm_; }

        auto& propertyTerm() const { return propertyTerm_; }

        auto& objectTerm() const { return objectTerm_; }

        auto& objectOperator() const { return objectOperator_; }

    protected:
        TermPtr subjectTerm_;
        TermPtr propertyTerm_;
        TermPtr objectTerm_;
        OperatorType objectOperator_;
    };

} // knowrob

#endif //KNOWROB_SEMWEB_TRIPLE_EXPRESSION_H
