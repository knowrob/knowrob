//
// Created by daniel on 07.04.23.
//

#include "knowrob/formulas/FramedLiteral.h"
#include "knowrob/terms/Constant.h"
#include "knowrob/Logger.h"

using namespace knowrob::semweb;

FramedLiteral::FramedLiteral(
            const TermPtr &subjectTerm,
            const TermPtr &propertyTerm,
            const TermPtr &objectTerm,
            OperatorType objectOperator,
            const std::string_view &graphName)
        : subjectTerm_(subjectTerm),
          propertyTerm_(propertyTerm),
          objectTerm_(objectTerm),
          beginTerm_(),
          endTerm_(),
          confidenceTerm_(),
          objectOperator_(objectOperator),
          beginOperator_(EQ),
          endOperator_(EQ),
          confidenceOperator_(EQ),
          graphTerm_(std::make_shared<StringTerm>(graphName.data()))
{
}

FramedLiteral::FramedLiteral(const StatementData &tripleData)
        : subjectTerm_(std::make_shared<StringTerm>(tripleData.subject)),
          propertyTerm_(std::make_shared<StringTerm>(tripleData.predicate)),
          beginTerm_(),
          endTerm_(),
          confidenceTerm_(),
          objectOperator_(EQ),
          beginOperator_(EQ),
          endOperator_(EQ),
          confidenceOperator_(EQ),
          graphTerm_(std::make_shared<StringTerm>(tripleData.graph))
{
    switch(tripleData.objectType) {
        case RDF_RESOURCE:
        case RDF_STRING_LITERAL:
            objectTerm_ = std::make_shared<StringTerm>(tripleData.object);
            break;
        case RDF_DOUBLE_LITERAL:
            objectTerm_ = std::make_shared<DoubleTerm>(tripleData.objectDouble);
            break;
        case RDF_BOOLEAN_LITERAL:
            objectTerm_ = std::make_shared<LongTerm>(tripleData.objectInteger);
            break;
        case RDF_INT64_LITERAL:
            objectTerm_ = std::make_shared<Integer32Term>(tripleData.objectInteger);
            break;
    }
    if(tripleData.confidence.has_value()) {
        confidenceTerm_ = std::make_shared<DoubleTerm>(tripleData.confidence.value());
    }
    if(tripleData.begin.has_value()) {
        beginTerm_ = std::make_shared<DoubleTerm>(tripleData.begin.value());
    }
    if(tripleData.end.has_value()) {
        endTerm_ = std::make_shared<DoubleTerm>(tripleData.end.value());
    }
}

FramedLiteral::FramedLiteral(const PredicatePtr &triplePredicate,
                             const std::string_view &graphName)
        : objectOperator_(EQ),
          beginOperator_(EQ),
          endOperator_(EQ),
          confidenceOperator_(EQ),
          beginTerm_(),
          endTerm_(),
          confidenceTerm_(),
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

void FramedLiteral::setMinConfidence(double limit)
{
    confidenceTerm_ = std::make_shared<DoubleTerm>(limit);
    confidenceOperator_ = LEQ;
}

void FramedLiteral::setMaxConfidence(double limit)
{
    confidenceTerm_ = std::make_shared<DoubleTerm>(limit);
    confidenceOperator_ = GEQ;
}

void FramedLiteral::setMinBegin(double limit)
{
    beginTerm_ = std::make_shared<DoubleTerm>(limit);
    beginOperator_ = GEQ;
}

void FramedLiteral::setMaxBegin(double limit)
{
    beginTerm_ = std::make_shared<DoubleTerm>(limit);
    beginOperator_ = LEQ;
}

void FramedLiteral::setMinEnd(double limit)
{
    endTerm_ = std::make_shared<DoubleTerm>(limit);
    endOperator_ = GEQ;
}

void FramedLiteral::setMaxEnd(double limit)
{
    endTerm_ = std::make_shared<DoubleTerm>(limit);
    endOperator_ = LEQ;
}

bool FramedLiteral::isGround() const
{
    return subjectTerm_->isGround() && propertyTerm()->isGround() && objectTerm_->isGround();
}
