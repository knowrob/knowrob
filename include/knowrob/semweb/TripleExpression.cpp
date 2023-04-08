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
            OperatorType objectOperator,
            const std::string_view &graphName)
        : subjectTerm_(subjectTerm),
          propertyTerm_(propertyTerm),
          objectTerm_(objectTerm),
          objectOperator_(objectOperator),
          beginOperator_(EQ),
          endOperator_(EQ),
          confidenceOperator_(EQ),
          graphTerm_(std::make_shared<StringTerm>(graphName.data()))
{
}

TripleExpression::TripleExpression(const PredicatePtr &triplePredicate,
                                   const std::string_view &graphName)
        : objectOperator_(EQ),
          beginOperator_(EQ),
          endOperator_(EQ),
          confidenceOperator_(EQ),
          graphTerm_(std::make_shared<StringTerm>(graphName.data()))
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

void TripleExpression::setMinConfidence(double limit)
{
    confidenceTerm_ = std::make_shared<DoubleTerm>(limit);
    confidenceOperator_ = LEQ;
}

void TripleExpression::setMaxConfidence(double limit)
{
    confidenceTerm_ = std::make_shared<DoubleTerm>(limit);
    confidenceOperator_ = GEQ;
}

void TripleExpression::setMinBegin(double limit)
{
    beginTerm_ = std::make_shared<DoubleTerm>(limit);
    beginOperator_ = GEQ;
}

void TripleExpression::setMaxBegin(double limit)
{
    beginTerm_ = std::make_shared<DoubleTerm>(limit);
    beginOperator_ = LEQ;
}

void TripleExpression::setMinEnd(double limit)
{
    endTerm_ = std::make_shared<DoubleTerm>(limit);
    endOperator_ = GEQ;
}

void TripleExpression::setMaxEnd(double limit)
{
    endTerm_ = std::make_shared<DoubleTerm>(limit);
    endOperator_ = LEQ;
}

bool TripleExpression::isGround() const
{
    return subjectTerm_->isGround() && propertyTerm()->isGround() && objectTerm_->isGround();
}
