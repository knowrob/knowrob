//
// Created by daniel on 07.04.23.
//

#include "knowrob/formulas/FramedLiteral.h"
#include "knowrob/terms/Constant.h"
#include "knowrob/Logger.h"

using namespace knowrob;

FramedLiteral::FramedLiteral(const LiteralPtr &literal, const ModalityFrame &modalityFrame)
: literal_(literal),
  modalityFrame_(modalityFrame)
{
    // FIXME: ensure that literal predicate is 2-ary
    if(literal_->predicate()->indicator()->arity() != 2) {
        //throw ...
    }
    subjectTerm_ = literal_->predicate()->arguments()[0];
    objectTerm_ = literal_->predicate()->arguments()[1];
    objectOperator_ = EQ;
    // TODO: support property variables in graph queries?
    //       The Predicate class used by literals does not allow unknown functors.
    propertyTerm_ = std::make_shared<StringTerm>(literal_->predicate()->indicator()->functor());

    auto epistemicOperator = modalityFrame_.epistemicOperator();
    if(epistemicOperator) {
        auto epistemicModality = (const EpistemicModality*) &epistemicOperator->modality();
        auto o_agent = epistemicModality->agent();
        if(o_agent) {
            agentTerm_ = std::make_shared<StringTerm>(o_agent.value());
        }
        if(epistemicOperator->isModalPossibility()) {
            confidenceTerm_ = std::make_shared<DoubleTerm>(0.0);
            confidenceOperator_ = LEQ;
        }
    }

    auto pastOperator = modalityFrame_.pastOperator();
    if(pastOperator) {
        auto pastModality = (const PastModality*) &pastOperator->modality();
        if(pastModality) {
            // TODO: allow quantified time intervals in past modality
            if(pastOperator->isModalPossibility()) {
                // include all triples before the current time point
                beginOperator_ = LT;
                beginTerm_ = std::make_shared<DoubleTerm>(TimePoint::now().value());
            }
            else if(pastOperator->isModalNecessity()) {
                // include only triples that were always true in the past
                // TODO: reconsider how this case should be encoded in triple expressions
                beginOperator_ = LEQ;
                beginTerm_ = std::make_shared<DoubleTerm>(0.0);
                endOperator_ = GEQ;
                endTerm_ = std::make_shared<DoubleTerm>(TimePoint::now().value());
            }
        }
        else {
            KB_WARN("unexpected temporal operator in graph query!");
        }
    }
}

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

/*
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
*/

std::shared_ptr<Term> FramedLiteral::subjectTerm() const
{
    return subjectTerm_;
}

std::shared_ptr<Term> FramedLiteral::objectTerm() const
{
    return objectTerm_;
}

FramedLiteral::OperatorType FramedLiteral::objectOperator() const
{
    return objectOperator_;
}

std::shared_ptr<Term> FramedLiteral::propertyTerm() const
{
    return propertyTerm_;
}

std::shared_ptr<Term> FramedLiteral::graphTerm() const
{
    return graphTerm_;
}

std::shared_ptr<Term> FramedLiteral::agentTerm() const
{
    return agentTerm_;
}

std::shared_ptr<Term> FramedLiteral::confidenceTerm() const
{
    return confidenceTerm_;
}

FramedLiteral::OperatorType FramedLiteral::confidenceOperator() const
{
    return confidenceOperator_;
}

std::shared_ptr<Term> FramedLiteral::beginTerm() const
{
    return beginTerm_;
}

std::shared_ptr<Term> FramedLiteral::endTerm() const
{
    return endTerm_;
}

FramedLiteral::OperatorType FramedLiteral::beginOperator() const
{
    return beginOperator_;
}

FramedLiteral::OperatorType FramedLiteral::endOperator() const
{
    return endOperator_;
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

void FramedLiteral::setBeginTerm(const TermPtr &beginTerm)
{
    beginTerm_ = beginTerm;
}

void FramedLiteral::setEndTerm(const TermPtr &endTerm)
{
    endTerm_ = endTerm;
}

void FramedLiteral::setAgentTerm(const std::string &agentTerm)
{
    agentTerm_ = std::make_shared<StringTerm>(agentTerm);
}

void FramedLiteral::setBeginOperator(OperatorType beginOperator)
{
    beginOperator_ = beginOperator;
}

void FramedLiteral::setEndOperator(OperatorType endOperator)
{
    endOperator_ = endOperator;
}

bool FramedLiteral::isGround() const
{
    return subjectTerm_->isGround() && propertyTerm()->isGround() && objectTerm_->isGround();
}

StatementData FramedLiteral::toStatementData() const
{
    StatementData data;
    // TODO: fill data
    return data;
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::FramedLiteral& l)
	{
	    // TODO: include more information when printing FramedLiteral
	    os << *l.propertyTerm() << '(';
	    os << *l.subjectTerm() << ", ";
	    os << *l.objectTerm();
	    os << ')';
	    return os;
	}
}
