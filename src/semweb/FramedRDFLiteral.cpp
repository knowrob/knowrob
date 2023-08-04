//
// Created by daniel on 07.04.23.
//

#include "knowrob/semweb/FramedRDFLiteral.h"
#include "knowrob/terms/Constant.h"
#include "knowrob/Logger.h"
#include "knowrob/queries/QueryError.h"

using namespace knowrob;

FramedRDFLiteral::FramedRDFLiteral(const LiteralPtr &literal, const ModalityFrame &modalityFrame)
: literal_(literal),
  modalityFrame_(modalityFrame)
{
    if(literal_->predicate()->indicator()->arity() != 2) {
        throw QueryError("RDF literals must be 2-ary, but "
                         "the literal '{}' is not.", *literal);
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
            if(pastModality->timeInterval().has_value()) {
                auto &ti = pastModality->timeInterval().value();
                if(ti.since().has_value()) {
                    beginTerm_ = std::make_shared<DoubleTerm>(ti.since().value().value());
                }
                if(ti.until().has_value()) {
                    endTerm_ = std::make_shared<DoubleTerm>(ti.until().value().value());
                }
            }
            if(!beginTerm_) {
                beginTerm_ = std::make_shared<DoubleTerm>(0.0);
            }
            if(!endTerm_) {
                endTerm_ = std::make_shared<DoubleTerm>(TimePoint::now().value());
            }
        }
        else {
            KB_WARN("unexpected temporal operator in graph query!");
        }
    }
}

FramedRDFLiteral::FramedRDFLiteral(
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
          confidenceOperator_(EQ),
          graphTerm_(std::make_shared<StringTerm>(graphName.data()))
{
}

FramedRDFLiteral::FramedRDFLiteral(const StatementData &tripleData)
        : subjectTerm_(std::make_shared<StringTerm>(tripleData.subject)),
          propertyTerm_(std::make_shared<StringTerm>(tripleData.predicate)),
          beginTerm_(),
          endTerm_(),
          confidenceTerm_(),
          objectOperator_(EQ),
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
FramedRDFLiteral::FramedRDFLiteral(const PredicatePtr &triplePredicate,
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

std::shared_ptr<Term> FramedRDFLiteral::subjectTerm() const
{
    return subjectTerm_;
}

std::shared_ptr<Term> FramedRDFLiteral::objectTerm() const
{
    return objectTerm_;
}

FramedRDFLiteral::OperatorType FramedRDFLiteral::objectOperator() const
{
    return objectOperator_;
}

std::shared_ptr<Term> FramedRDFLiteral::propertyTerm() const
{
    return propertyTerm_;
}

std::shared_ptr<Term> FramedRDFLiteral::graphTerm() const
{
    return graphTerm_;
}

std::shared_ptr<Term> FramedRDFLiteral::agentTerm() const
{
    return agentTerm_;
}

std::shared_ptr<Term> FramedRDFLiteral::confidenceTerm() const
{
    return confidenceTerm_;
}

FramedRDFLiteral::OperatorType FramedRDFLiteral::confidenceOperator() const
{
    return confidenceOperator_;
}

std::shared_ptr<Term> FramedRDFLiteral::beginTerm() const
{
    return beginTerm_;
}

std::shared_ptr<Term> FramedRDFLiteral::endTerm() const
{
    return endTerm_;
}

void FramedRDFLiteral::setMinConfidence(double limit)
{
    confidenceTerm_ = std::make_shared<DoubleTerm>(limit);
    confidenceOperator_ = LEQ;
}

void FramedRDFLiteral::setMaxConfidence(double limit)
{
    confidenceTerm_ = std::make_shared<DoubleTerm>(limit);
    confidenceOperator_ = GEQ;
}

void FramedRDFLiteral::setBeginTerm(const TermPtr &beginTerm)
{
    beginTerm_ = beginTerm;
}

void FramedRDFLiteral::setEndTerm(const TermPtr &endTerm)
{
    endTerm_ = endTerm;
}

void FramedRDFLiteral::setAgentTerm(const std::string &agentTerm)
{
    agentTerm_ = std::make_shared<StringTerm>(agentTerm);
}

bool FramedRDFLiteral::isGround() const
{
    return subjectTerm_->isGround() && propertyTerm()->isGround() && objectTerm_->isGround();
}

static inline const char* readStringConstant(const TermPtr &term)
{ return std::static_pointer_cast<StringTerm>(term)->value().c_str(); }

static inline double readDoubleConstant(const TermPtr &term)
{ return std::static_pointer_cast<DoubleTerm>(term)->value(); }

StatementData FramedRDFLiteral::toStatementData() const
{
    StatementData data;
    if(!isGround()) {
        throw QueryError("Only ground literals can be mapped to StatementData, but "
                         "the literal '{}' has variables.", *this);
    }
    data.subject = readStringConstant(subjectTerm_);
    data.predicate = readStringConstant(propertyTerm_);
    switch(objectTerm_->type()) {
        case TermType::STRING:
            data.object = ((StringTerm*)objectTerm_.get())->value().c_str();
            // TODO: RDF_RESOURCE or RDF_STRING_LITERAL?
            data.objectType = RDF_RESOURCE;
            break;
        case TermType::DOUBLE:
            data.objectDouble = ((DoubleTerm*)objectTerm_.get())->value();
            data.objectType = RDF_DOUBLE_LITERAL;
            break;
        case TermType::INT32:
            data.objectInteger = ((Integer32Term*)objectTerm_.get())->value();
            data.objectType = RDF_INT64_LITERAL;
            break;
        case TermType::LONG:
            data.objectInteger = ((LongTerm*)objectTerm_.get())->value();
            data.objectType = RDF_INT64_LITERAL;
            break;
        case TermType::VARIABLE:
            KB_WARN("Literal has a variable as an argument in '{}' which is not allowed.", *this);
            break;
        case TermType::PREDICATE:
            KB_WARN("Literal has a predicate as an argument in '{}' which is not allowed.", *this);
            break;
        case TermType::LIST:
            KB_WARN("Literal has a list as an argument in '{}' which is not allowed.", *this);
            break;
    }
    if(agentTerm_)      data.agent = readStringConstant(agentTerm_);
    if(graphTerm_)      data.graph = readStringConstant(graphTerm_);
    if(confidenceTerm_) data.confidence = readDoubleConstant(confidenceTerm_);

    // handle temporal modality
    if(modalityFrame_.pastOperator()) {
        if(modalityFrame_.pastOperator()->isModalNecessity()) {
            data.temporalOperator = TemporalOperator::ALL_PAST;
        }
        else {
            data.temporalOperator = TemporalOperator::SOME_PAST;
        }
    }
    else if(beginTerm_ || endTerm_) {
        // implicit use of H operator when only time interval was specified
        data.temporalOperator = TemporalOperator::ALL_PAST;
    }
    if(beginTerm_) {
        data.begin = readDoubleConstant(beginTerm_);
    }
    if(endTerm_) {
        data.end = readDoubleConstant(endTerm_);
    }

    return data;
}

namespace std {
	std::ostream& operator<<(std::ostream& os, const knowrob::FramedRDFLiteral& l)
	{
	    os << *l.propertyTerm();
        if(l.modalityFrame().hasValue()) {
            os << l.modalityFrame();
        }
        os << '(';
	    os << *l.subjectTerm() << ", ";
	    os << *l.objectTerm();
	    os << ')';
	    return os;
	}
}
