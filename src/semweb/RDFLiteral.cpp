//
// Created by daniel on 07.04.23.
//

#include <utility>

#include "knowrob/semweb/RDFLiteral.h"
#include "knowrob/Logger.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/modalities/BeliefModality.h"
#include "knowrob/terms/Atomic.h"
#include "knowrob/terms/Numeric.h"
#include "knowrob/terms/Atom.h"
#include "knowrob/terms/IRIAtom.h"
#include "knowrob/terms/String.h"
#include "knowrob/terms/Blank.h"

using namespace knowrob;

RDFLiteral::RDFLiteral(const FramedTriple &data, bool isNegated)
: FirstOrderLiteral(getRDFPredicate(data), isNegated),
  subjectTerm_(predicate_->arguments()[0]),
  propertyTerm_(predicate_->arguments()[1]),
  objectTerm_(predicate_->arguments()[2]),
  objectOperator_(EQ),
  epistemicOperator_(data.epistemicOperator()),
  temporalOperator_(data.temporalOperator())
{
    if(data.confidence().has_value()) {
        confidenceTerm_ = std::make_shared<Double>(data.confidence().value());
    }
    if(data.begin().has_value()) {
        beginTerm_ = std::make_shared<Double>(data.begin().value());
    }
    if(data.end().has_value()) {
        endTerm_ = std::make_shared<Double>(data.end().value());
    }
    if(data.graph()) {
        graphTerm_ = getGraphTerm(data.graph().value());
    }
    if(data.agent()) {
        agentTerm_ = Atom::Tabled(data.agent().value());
    }
}

RDFLiteral::RDFLiteral(const PredicatePtr &predicate, bool isNegated, const GraphSelector &selector)
: FirstOrderLiteral(getRDFPredicate(predicate), isNegated),
  subjectTerm_(predicate_->arguments()[0]),
  propertyTerm_(predicate_->arguments()[1]),
  objectTerm_(predicate_->arguments()[2]),
  objectOperator_(EQ),
  epistemicOperator_(selector.epistemicOperator),
  temporalOperator_(selector.temporalOperator)
{
    if(selector.confidence.has_value()) {
        confidenceTerm_ = std::make_shared<Double>(selector.confidence.value());
    }
    if(selector.begin.has_value()) {
        beginTerm_ = std::make_shared<Double>(selector.begin.value());
    }
    if(selector.end.has_value()) {
        endTerm_ = std::make_shared<Double>(selector.end.value());
    }
    if(selector.graph) {
        graphTerm_ = getGraphTerm(selector.graph);
    }
    if(selector.agent) {
        agentTerm_ = Atom::Tabled(selector.agent.value()->iri());
    }
}

RDFLiteral::RDFLiteral(const RDFLiteral &other, const Substitution &sub)
: FirstOrderLiteral(other, sub),
  subjectTerm_(predicate_->arguments()[0]),
  propertyTerm_(predicate_->arguments()[1]),
  objectTerm_(predicate_->arguments()[2]),
  graphTerm_(other.graphTerm_),
  agentTerm_(other.agentTerm_),
  confidenceTerm_(other.confidenceTerm_),
  beginTerm_(other.beginTerm_),
  endTerm_(other.endTerm_),
  objectOperator_(EQ),
  epistemicOperator_(other.epistemicOperator_),
  temporalOperator_(other.temporalOperator_)
{
    // todo: substitute other variables of RDFLiteral too!
}

RDFLiteral::RDFLiteral(
            const TermPtr &s,
            const TermPtr &p,
            const TermPtr &o,
            bool isNegated,
            const GraphSelector &selector)
: FirstOrderLiteral(getRDFPredicate(s,p,o), isNegated),
  subjectTerm_(s),
  propertyTerm_(p),
  objectTerm_(o),
  objectOperator_(EQ)
{
	if(selector.epistemicOperator) {
		epistemicOperator_ = selector.epistemicOperator;
		if(selector.agent.has_value()) {
			agentTerm_ = Atom::Tabled(selector.agent.value()->iri());
		}
		if(selector.confidence.has_value()) {
			confidenceTerm_ = std::make_shared<Double>(selector.confidence.value());
		}
	}

	if(selector.temporalOperator) {
		temporalOperator_ = selector.temporalOperator;
		if(selector.begin.has_value()) {
			beginTerm_ = std::make_shared<Double>(selector.begin.value());
		}
		if(selector.end.has_value()) {
			endTerm_ = std::make_shared<Double>(selector.end.value());
		}
	}
}

std::shared_ptr<Term> RDFLiteral::getGraphTerm(const std::string_view &graphName)
{
    static std::map<std::string, TermPtr, std::less<>> graphTerms;
    if(!graphName.empty()) {
        auto it = graphTerms.find(graphName);
        if(it == graphTerms.end()) {
            auto graphTerm = Atom::Tabled(graphName.data());
            graphTerms[graphName.data()] = graphTerm;
            return graphTerm;
        }
        else {
            return it->second;
        }
    }
    return {};
}

std::shared_ptr<Predicate> RDFLiteral::getRDFPredicate(const TermPtr &s, const TermPtr &p, const TermPtr &o)
{
    return std::make_shared<Predicate>("triple", std::vector<TermPtr>({s,p,o}));
}

std::shared_ptr<Predicate> RDFLiteral::getRDFPredicate(const PredicatePtr &predicate)
{
	if(predicate->arity()==3 && predicate->functor()->stringForm() == "triple") {
		return predicate;
	}
	else if(predicate->arity()==2) {
		return getRDFPredicate(predicate->arguments()[0],
							   predicate->functor(),
							   predicate->arguments()[1]);
	}
	else {
		throw QueryError("RDF literal can only be constructed from 2-ary predicates but {} is not.", *predicate);
	}
}

std::shared_ptr<Predicate> RDFLiteral::getRDFPredicate(const FramedTriple &data)
{
    TermPtr s,p,o;
    p = IRIAtom::Tabled(data.predicate());
    if (data.isSubjectBlank()) {
		s = Blank::Tabled(data.subject());
	} else {
		s = IRIAtom::Tabled(data.subject());
    }
    if(data.isObjectBlank()) {
    	o = Blank::Tabled(data.valueAsString());
    } else {
    	o = Atomic::makeTripleValue(data);
    }
    return std::make_shared<Predicate>("triple", std::vector<TermPtr>({s,p,o}));
}

std::shared_ptr<Term> RDFLiteral::subjectTerm() const
{
    return subjectTerm_;
}

std::shared_ptr<Term> RDFLiteral::objectTerm() const
{
    return objectTerm_;
}

RDFLiteral::OperatorType RDFLiteral::objectOperator() const
{
    return objectOperator_;
}

std::shared_ptr<Term> RDFLiteral::propertyTerm() const
{
    return propertyTerm_;
}

std::shared_ptr<Term> RDFLiteral::graphTerm() const
{
    return graphTerm_;
}

void RDFLiteral::setGraphName(const std::string_view &graphName)
{
    graphTerm_ = getGraphTerm(graphName);
}

std::shared_ptr<Term> RDFLiteral::agentTerm() const
{
    return agentTerm_;
}

std::shared_ptr<Term> RDFLiteral::confidenceTerm() const
{
    return confidenceTerm_;
}

std::shared_ptr<Term> RDFLiteral::beginTerm() const
{
    return beginTerm_;
}

std::shared_ptr<Term> RDFLiteral::endTerm() const
{
    return endTerm_;
}

uint32_t RDFLiteral::numVariables() const
{
    int varCounter=0;
    for(auto &t : {
        subjectTerm_,
        propertyTerm_,
        objectTerm_,
        graphTerm_,
        agentTerm_,
        beginTerm_,
        endTerm_,
        confidenceTerm_ })
    {
        if(t && t->termType() == TermType::VARIABLE) varCounter+=1;
    }
    return varCounter;
}

static inline std::string_view readStringConstant(const TermPtr &term)
{ return std::static_pointer_cast<Atomic>(term)->stringForm(); }

static inline double readDoubleConstant(const TermPtr &term)
{ return std::static_pointer_cast<Numeric>(term)->asDouble(); }

bool RDFLiteral::toStatementData(FramedTriple &data) const
{
    if(numVariables()>0) {
        KB_WARN("Only ground literals can be mapped to StatementData, but "
                         "the literal '{}' has variables.", *this);
		return false;
    }
    if(isNegated()) {
        KB_WARN("Only positive literals can be mapped to StatementData, but "
                         "the literal '{}' is negative.", *this);
		return false;
    }
    data.setSubject(readStringConstant(subjectTerm_));
    data.setPredicate(readStringConstant(propertyTerm_));

    if(objectTerm_->isNumeric()) {
    	auto numeric = std::static_pointer_cast<Numeric>(objectTerm_);
    	data.setXSDValue(numeric->stringForm(), numeric->xsdType());
	} else if (objectTerm_->isAtom()) {
		auto atom = std::static_pointer_cast<Atomic>(objectTerm_);
		if (atom->isIRI()) {
			data.setObjectIRI(atom->stringForm());
		} else {
			data.setStringValue(atom->stringForm());
		}
	} else if (objectTerm_->isString()) {
    	data.setStringValue(std::static_pointer_cast<Atomic>(objectTerm_)->stringForm());
	} else {
		KB_WARN("Literal has a non-atomic argument in '{}' which is not allowed.", *this);
		return false;
	}

    if(graphTerm_) {
		data.setGraph(readStringConstant(graphTerm_));
	}

    // handle epistemic modality
    if(epistemicOperator_) {
		data.setEpistemicOperator(epistemicOperator_.value());
    }
    if(agentTerm_) {
        data.setAgent(readStringConstant(agentTerm_));
    }
    if(confidenceTerm_) {
        data.setConfidence(readDoubleConstant(confidenceTerm_));
    }

    // handle temporal modality
    if(temporalOperator_) {
        data.setTemporalOperator(temporalOperator_.value());
    }
    if(beginTerm_) {
        data.setBegin(readDoubleConstant(beginTerm_));
    }
    if(endTerm_) {
        data.setEnd(readDoubleConstant(endTerm_));
    }

    return true;
}

void RDFLiteralContainer::push_back(const RDFLiteralPtr &triple) {
	statements_.emplace_back(triple);
	auto &data = data_.emplace_back();
	data.ptr = new FramedTripleView();
	data.owned = true;
	if(!triple->toStatementData(*data.ptr)) {
		data_.pop_back();
	}
}
