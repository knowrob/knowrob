/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/semweb/FramedTriplePattern.h"
#include "knowrob/Logger.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/modalities/BeliefModality.h"
#include "knowrob/terms/Atomic.h"
#include "knowrob/terms/Numeric.h"
#include "knowrob/terms/Atom.h"
#include "knowrob/terms/IRIAtom.h"
#include "knowrob/terms/Blank.h"

using namespace knowrob;

FramedTriplePattern::FramedTriplePattern(const FramedTriple &triple, bool isNegated)
		: FirstOrderLiteral(getRDFPredicate(triple), isNegated),
		  subjectTerm_(predicate_->arguments()[0]),
		  propertyTerm_(predicate_->arguments()[1]),
		  objectTerm_(predicate_->arguments()[2]),
		  objectOperator_(EQ),
		  epistemicOperator_(triple.epistemicOperator()),
		  temporalOperator_(triple.temporalOperator()) {
	if (triple.confidence().has_value()) {
		confidenceTerm_ = std::make_shared<Double>(triple.confidence().value());
	}
	if (triple.begin().has_value()) {
		beginTerm_ = std::make_shared<Double>(triple.begin().value());
	}
	if (triple.end().has_value()) {
		endTerm_ = std::make_shared<Double>(triple.end().value());
	}
	if (triple.graph()) {
		graphTerm_ = getGraphTerm(triple.graph().value());
	}
	if (triple.agent()) {
		agentTerm_ = Atom::Tabled(triple.agent().value());
	}
}

FramedTriplePattern::FramedTriplePattern(const PredicatePtr &pred, bool isNegated)
		: FirstOrderLiteral(getRDFPredicate(pred), isNegated),
		  subjectTerm_(predicate_->arguments()[0]),
		  propertyTerm_(predicate_->arguments()[1]),
		  objectTerm_(predicate_->arguments()[2]),
		  objectOperator_(EQ) {
}

FramedTriplePattern::FramedTriplePattern(const TermPtr &s, const TermPtr &p, const TermPtr &o, bool isNegated)
		: FirstOrderLiteral(getRDFPredicate(s,p,o), isNegated),
		  subjectTerm_(s),
		  propertyTerm_(p),
		  objectTerm_(o),
		  objectOperator_(EQ) {
}

void FramedTriplePattern::setTripleFrame(const GraphSelector &frame) {
	epistemicOperator_ = frame.epistemicOperator;
	temporalOperator_ = frame.temporalOperator;
	if (frame.confidence.has_value()) {
		confidenceTerm_ = std::make_shared<Double>(frame.confidence.value());
	}
	if (frame.begin.has_value()) {
		beginTerm_ = std::make_shared<Double>(frame.begin.value());
	}
	if (frame.end.has_value()) {
		endTerm_ = std::make_shared<Double>(frame.end.value());
	}
	if (frame.graph) {
		graphTerm_ = getGraphTerm(frame.graph);
	}
	if (frame.agent) {
		agentTerm_ = Atom::Tabled(frame.agent.value()->iri());
	}
}

std::shared_ptr<Atom> FramedTriplePattern::getGraphTerm(const std::string_view &graphName) {
	static std::map<std::string, AtomPtr, std::less<>> graphTerms;
	if (!graphName.empty()) {
		auto it = graphTerms.find(graphName);
		if (it == graphTerms.end()) {
			auto graphTerm = Atom::Tabled(graphName.data());
			graphTerms[graphName.data()] = graphTerm;
			return graphTerm;
		} else {
			return it->second;
		}
	}
	return {};
}

std::shared_ptr<Predicate> FramedTriplePattern::getRDFPredicate(const TermPtr &s, const TermPtr &p, const TermPtr &o) {
	return std::make_shared<Predicate>("triple", std::vector<TermPtr>({s, p, o}));
}

std::shared_ptr<Predicate> FramedTriplePattern::getRDFPredicate(const PredicatePtr &predicate) {
	if (predicate->arity() == 3 && predicate->functor()->stringForm() == "triple") {
		return predicate;
	} else if (predicate->arity() == 2) {
		return getRDFPredicate(predicate->arguments()[0],
							   predicate->functor(),
							   predicate->arguments()[1]);
	} else {
		throw QueryError("RDF literal can only be constructed from 2-ary predicates but {} is not.", *predicate);
	}
}

std::shared_ptr<Predicate> FramedTriplePattern::getRDFPredicate(const FramedTriple &data) {
	TermPtr s, p, o;
	p = IRIAtom::Tabled(data.predicate());
	if (data.isSubjectBlank()) {
		s = Blank::Tabled(data.subject());
	} else {
		s = IRIAtom::Tabled(data.subject());
	}
	if (data.isObjectBlank()) {
		o = Blank::Tabled(data.valueAsString());
	} else {
		o = Atomic::makeTripleValue(data);
	}
	return std::make_shared<Predicate>("triple", std::vector<TermPtr>({s, p, o}));
}

uint32_t FramedTriplePattern::numVariables() const {
	int varCounter = 0;
	for (auto &t: {
			subjectTerm_,
			propertyTerm_,
			objectTerm_,
			*graphTerm_,
			*agentTerm_,
			*beginTerm_,
			*endTerm_,
			*confidenceTerm_}) {
		if (t && t->termType() == TermType::VARIABLE) varCounter += 1;
	}
	return varCounter;
}

static inline std::string_view readStringConstant(const TermPtr &term) {
	return std::static_pointer_cast<Atomic>(term)->stringForm();
}

bool FramedTriplePattern::toStatementData(FramedTriple &data) const {
	if (numVariables() > 0) {
		KB_WARN("Only ground literals can be mapped to StatementData, but "
				"the literal '{}' has variables.", *this);
		return false;
	}
	if (isNegated()) {
		KB_WARN("Only positive literals can be mapped to StatementData, but "
				"the literal '{}' is negative.", *this);
		return false;
	}
	data.setSubject(std::static_pointer_cast<Atomic>(subjectTerm_)->stringForm());
	data.setPredicate(std::static_pointer_cast<Atomic>(propertyTerm_)->stringForm());

	if (objectTerm_->isNumeric()) {
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

	if (graphTerm_) {
		data.setGraph(graphTerm_.grounded()->stringForm());
	}

	// handle epistemic modality
	if (epistemicOperator_) {
		data.setEpistemicOperator(epistemicOperator_.value());
	}
	if (agentTerm_) {
		data.setAgent(agentTerm_.grounded()->stringForm());
	}
	if (confidenceTerm_) {
		data.setConfidence(confidenceTerm_.grounded()->asDouble());
	}

	// handle temporal modality
	if (temporalOperator_) {
		data.setTemporalOperator(temporalOperator_.value());
	}
	if (beginTerm_) {
		data.setBegin(beginTerm_.grounded()->asDouble());
	}
	if (endTerm_) {
		data.setEnd(endTerm_.grounded()->asDouble());
	}

	return true;
}

void RDFLiteralContainer::push_back(const FramedTriplePatternPtr &triple) {
	statements_.emplace_back(triple);
	auto &data = data_.emplace_back();
	data.ptr = new FramedTripleView();
	data.owned = true;
	if (!triple->toStatementData(*data.ptr)) {
		data_.pop_back();
	}
}

namespace knowrob {
	FramedTriplePatternPtr applyBindings(const FramedTriplePatternPtr &pat, const Substitution &bindings) {
		bool hasChanges = false;

		auto subject = applyBindings(pat->subjectTerm(), bindings);
		if (subject != pat->subjectTerm()) hasChanges = true;

		auto property = applyBindings(pat->propertyTerm(), bindings);
		if (property != pat->propertyTerm()) hasChanges = true;

		auto object = applyBindings(pat->objectTerm(), bindings);
		if (object != pat->objectTerm()) hasChanges = true;

		auto graph = pat->graphTerm() ? applyBindings(*pat->graphTerm(), bindings) : nullptr;
		if (graph && graph != *pat->graphTerm()) hasChanges = true;

		auto agent = pat->agentTerm() ? applyBindings(*pat->agentTerm(), bindings) : nullptr;
		if (agent && agent != *pat->agentTerm()) hasChanges = true;

		auto confidence = pat->confidenceTerm() ? applyBindings(*pat->confidenceTerm(), bindings) : nullptr;
		if (confidence && confidence != *pat->confidenceTerm()) hasChanges = true;

		auto begin = pat->beginTerm() ? applyBindings(*pat->beginTerm(), bindings) : nullptr;
		if (begin && begin != *pat->beginTerm()) hasChanges = true;

		auto end = pat->endTerm() ? applyBindings(*pat->endTerm(), bindings) : nullptr;
		if (end && end != *pat->endTerm()) hasChanges = true;

		if(!hasChanges) return pat;

		auto patInstance = std::make_shared<FramedTriplePattern>(
			subject, property, object, pat->isNegated());
		patInstance->setObjectOperator(pat->objectOperator());
		if (graph) patInstance->setGraphTerm(groundable<Atom>::cast(graph));
		if (agent) patInstance->setAgentTerm(groundable<Atom>::cast(agent));
		if (confidence) patInstance->setConfidenceTerm(groundable<Double>::cast(confidence));
		if (begin) patInstance->setBeginTerm(groundable<Double>::cast(begin));
		if (end) patInstance->setEndTerm(groundable<Double>::cast(end));
		return patInstance;
	}
}
