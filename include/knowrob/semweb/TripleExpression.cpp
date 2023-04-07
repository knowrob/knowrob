//
// Created by daniel on 07.04.23.
//

#include "TripleExpression.h"
#include "knowrob/terms/Constant.h"
#include "knowrob/Logger.h"

using namespace knowrob::semweb;

TripleExpression::TripleExpression(
            const TermPtr &subjectTerm,
            const TermPtr &propertyTerm,
            const TermPtr &objectTerm,
            OperatorType objectOperator)
        : subjectTerm_(subjectTerm),
          propertyTerm_(propertyTerm),
          objectTerm_(objectTerm),
          objectOperator_(objectOperator)
{
}

TripleExpression::TripleExpression(const PredicatePtr &triplePredicate)
        : objectOperator_(EQ)
{
    if(triplePredicate->indicator()->arity()!=2) {
        // TODO: throw exception class
        throw std::runtime_error("arity is not 2");
    }
    subjectTerm_ = triplePredicate->arguments()[0];
    propertyTerm_ = std::make_shared<StringTerm>(triplePredicate->indicator()->functor());

    auto ot = triplePredicate->arguments()[1];
    if(ot->type() == TermType::PREDICATE) {
        auto p = (Predicate*)ot.get();
        if(p->indicator()->arity()!=2) {
            // TODO: throw exception class
            throw std::runtime_error("the operator is not unary");
        }
        auto &operatorSymbol = p->indicator()->functor();
        if(operatorSymbol == "=") objectOperator_=EQ;
        else if(operatorSymbol == "<") objectOperator_=LT;
        else if(operatorSymbol == ">") objectOperator_=GT;
        else if(operatorSymbol == "<=") objectOperator_=LEQ;
        else if(operatorSymbol == ">=") objectOperator_=GEQ;
        else KB_WARN("unknown operator {}", operatorSymbol);
        objectTerm_ = p->arguments()[0];
    }
    else {
        objectTerm_ = ot;
    }
}
