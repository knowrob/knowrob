/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/triples/FramedTriplePattern.h"
#include "knowrob/Logger.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/modalities/BeliefModality.h"
#include "knowrob/terms/Atomic.h"
#include "knowrob/terms/Numeric.h"
#include "knowrob/terms/Atom.h"
#include "knowrob/terms/IRIAtom.h"
#include "knowrob/terms/Blank.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

FramedTriplePattern::FramedTriplePattern(const FramedTriple &triple, bool isNegated)
		: FirstOrderLiteral(getRDFPredicate(triple), isNegated),
		  subjectTerm_(predicate_->arguments()[0]),
		  propertyTerm_(predicate_->arguments()[1]),
		  objectTerm_(predicate_->arguments()[2]),
		  objectOperator_(EQ),
		  isOptional_(false) {
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
	if (triple.isOccasional()) {
		isOccasional_ = Numeric::trueAtom();
	}
	if (triple.isUncertain()) {
		isUncertain_ = Numeric::trueAtom();
	}
}

FramedTriplePattern::FramedTriplePattern(const PredicatePtr &pred, bool isNegated)
		: FirstOrderLiteral(getRDFPredicate(pred), isNegated),
		  subjectTerm_(predicate_->arguments()[0]),
		  propertyTerm_(predicate_->arguments()[1]),
		  objectTerm_(predicate_->arguments()[2]),
		  objectOperator_(EQ),
		  isOptional_(false) {
}

FramedTriplePattern::FramedTriplePattern(const TermPtr &s, const TermPtr &p, const TermPtr &o, bool isNegated)
		: FirstOrderLiteral(getRDFPredicate(s, p, o), isNegated),
		  subjectTerm_(s),
		  propertyTerm_(p),
		  objectTerm_(o),
		  objectOperator_(EQ),
		  isOptional_(false) {
}

void FramedTriplePattern::setTripleFrame(const GraphSelector &frame) {
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
	if (frame.epistemicOperator.has_value() && frame.epistemicOperator.value() == EpistemicOperator::BELIEF) {
		isUncertain_ = Numeric::trueAtom();
	}
	if (frame.temporalOperator.has_value() && frame.temporalOperator.value() == TemporalOperator::SOMETIMES) {
		isOccasional_ = Numeric::trueAtom();
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

std::vector<VariablePtr> FramedTriplePattern::getVariables() const {
	std::vector<VariablePtr> vars;
	for (auto &t: {
			subjectTerm_,
			propertyTerm_,
			objectTerm_,
			*graphTerm_,
			*agentTerm_,
			*beginTerm_,
			*endTerm_,
			*isUncertain_,
			*isOccasional_,
			*confidenceTerm_}) {
		if (t && t->termType() == TermType::VARIABLE) vars.push_back(std::static_pointer_cast<Variable>(t));
	}
	return vars;
}

uint32_t FramedTriplePattern::numVariables() const {
	return getVariables().size();
}

bool
FramedTriplePattern::instantiateInto(FramedTriple &triple, const std::shared_ptr<const Substitution> &bindings) const {
	// return a flag that indicates if s/p/o were assigned successfully.
	bool hasMissingSPO = false;
	// handle subject
	if (subjectTerm_) {
		auto &s = subjectTerm_->isVariable() ?
				  bindings->get(std::static_pointer_cast<Variable>(subjectTerm_)->name()) :
				  subjectTerm_;
		if (!s) {
			hasMissingSPO = true;
		} else if (s->isBlank()) {
			triple.setSubjectBlank(std::static_pointer_cast<Blank>(s)->stringForm());
		} else if (s->termType() == TermType::ATOMIC) {
			triple.setSubject(std::static_pointer_cast<Atomic>(s)->stringForm());
		} else {
			hasMissingSPO = true;
		}
	} else {
		hasMissingSPO = true;
	}
	// handle property
	if (propertyTerm_) {
		auto &p = propertyTerm_->isVariable() ?
				  bindings->get(std::static_pointer_cast<Variable>(propertyTerm_)->name()) :
				  propertyTerm_;
		if (p && p->termType() == TermType::ATOMIC) {
			triple.setPredicate(std::static_pointer_cast<Atomic>(p)->stringForm());
		} else {
			hasMissingSPO = true;
		}
	} else {
		hasMissingSPO = true;
	}
	// handle object
	if (objectTerm_) {
		auto &o = objectTerm_->isVariable() ?
				  bindings->get(std::static_pointer_cast<Variable>(objectTerm_)->name()) :
				  objectTerm_;
		if (!o) {
			hasMissingSPO = true;
		} else if (o->isNumeric()) {
			auto numeric = std::static_pointer_cast<Numeric>(o);
			triple.setXSDValue(numeric->stringForm(), numeric->xsdType());
		} else if (o->termType() == TermType::ATOMIC) {
			auto atom = std::static_pointer_cast<Atomic>(o);
			if (atom->isIRI()) {
				triple.setObjectIRI(atom->stringForm());
			} else if (atom->isBlank()) {
				triple.setObjectBlank(atom->stringForm());
			} else {
				triple.setStringValue(atom->stringForm());
			}
		} else {
			hasMissingSPO = true;
		}
	} else {
		hasMissingSPO = true;
	}
	// handle optional properties
	if (graphTerm_) {
		auto &g = graphTerm_->isVariable() ?
				  bindings->get(std::static_pointer_cast<Variable>(*graphTerm_)->name()) :
				  *graphTerm_;
		if (g && g->termType() == TermType::ATOMIC) {
			triple.setGraph(std::static_pointer_cast<Atomic>(g)->stringForm());
		}
	}
	if (agentTerm_) {
		auto &a = agentTerm_->isVariable() ?
				  bindings->get(std::static_pointer_cast<Variable>(*agentTerm_)->name()) :
				  *agentTerm_;
		if (a && a->termType() == TermType::ATOMIC) {
			triple.setAgent(std::static_pointer_cast<Atomic>(a)->stringForm());
		}
	}
	if (confidenceTerm_) {
		auto &c = confidenceTerm_->isVariable() ?
				  bindings->get(std::static_pointer_cast<Variable>(*confidenceTerm_)->name()) :
				  *confidenceTerm_;
		if (c && c->isNumeric()) {
			triple.setConfidence(std::static_pointer_cast<Numeric>(c)->asDouble());
		}
	}
	if (beginTerm_) {
		auto &b = beginTerm_->isVariable() ?
				  bindings->get(std::static_pointer_cast<Variable>(*beginTerm_)->name()) :
				  *beginTerm_;
		if (b && b->isNumeric()) {
			triple.setBegin(std::static_pointer_cast<Numeric>(b)->asDouble());
		}
	}
	if (endTerm_) {
		auto &e = endTerm_->isVariable() ?
				  bindings->get(std::static_pointer_cast<Variable>(*endTerm_)->name()) :
				  *endTerm_;
		if (e && e->isNumeric()) {
			triple.setEnd(std::static_pointer_cast<Numeric>(e)->asDouble());
		}
	}
	if (isOccasional_) {
		auto &o = isOccasional_->isVariable() ?
				  bindings->get(std::static_pointer_cast<Variable>(*isOccasional_)->name()) :
				  *isOccasional_;
		if (o && o->isNumeric()) {
			triple.setIsOccasional(std::static_pointer_cast<Numeric>(o)->asBoolean());
		}
	}
	if (isUncertain_) {
		auto &u = isUncertain_->isVariable() ?
				  bindings->get(std::static_pointer_cast<Variable>(*isUncertain_)->name()) :
				  *isUncertain_;
		if (u && u->isNumeric()) {
			triple.setIsUncertain(std::static_pointer_cast<Numeric>(u)->asBoolean());
		}
	}

	return !hasMissingSPO;
}

void TriplePatternContainer::push_back(const FramedTriplePatternPtr &q) {
	statements_.emplace_back(q);
	auto &data = data_.emplace_back();
	data.ptr = new FramedTripleView();
	data.owned = true;
	if (!q->instantiateInto(*data.ptr)) {
		data_.pop_back();
	}
}

semweb::TripleContainer::ConstGenerator TriplePatternContainer::cgenerator() const {
	return [this, i = 0]() mutable -> const FramedTriplePtr * {
		if (i < data_.size()) return &data_[i++];
		return nullptr;
	};
}

semweb::MutableTripleContainer::MutableGenerator TriplePatternContainer::generator() {
	return [this, i = 0]() mutable -> FramedTriplePtr * {
		if (i < data_.size()) return &data_[i++];
		return nullptr;
	};
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

		auto occasional = pat->isOccasionalTerm() ? applyBindings(*pat->isOccasionalTerm(), bindings) : nullptr;
		if (occasional && occasional != *pat->isOccasionalTerm()) hasChanges = true;

		auto uncertain = pat->isUncertainTerm() ? applyBindings(*pat->isUncertainTerm(), bindings) : nullptr;
		if (uncertain && uncertain != *pat->isUncertainTerm()) hasChanges = true;

		if (!hasChanges) return pat;

		auto patInstance = std::make_shared<FramedTriplePattern>(
				subject, property, object, pat->isNegated());
		patInstance->setObjectOperator(pat->objectOperator());
		if (graph) patInstance->setGraphTerm(groundable<Atom>::cast(graph));
		if (agent) patInstance->setAgentTerm(groundable<Atom>::cast(agent));
		if (confidence) patInstance->setConfidenceTerm(groundable<Double>::cast(confidence));
		if (begin) patInstance->setBeginTerm(groundable<Double>::cast(begin));
		if (end) patInstance->setEndTerm(groundable<Double>::cast(end));
		if (occasional) patInstance->setIsOccasionalTerm(groundable<Numeric>::cast(occasional));
		if (uncertain) patInstance->setIsUncertainTerm(groundable<Numeric>::cast(uncertain));
		return patInstance;
	}
}

namespace knowrob::py {
	template<>
	void createType<FramedTriplePattern>() {
		using namespace boost::python;
		class_<FramedTriplePattern, std::shared_ptr<FramedTriplePattern>, bases<FirstOrderLiteral>>
				("FramedTriplePattern", init<const FramedTriple &, bool>())
				.def(init<const FramedTriple &>())
				.def("subjectTerm", &FramedTriplePattern::subjectTerm, return_value_policy<return_by_value>())
				.def("propertyTerm", &FramedTriplePattern::propertyTerm, return_value_policy<return_by_value>())
				.def("objectTerm", &FramedTriplePattern::objectTerm, return_value_policy<return_by_value>())
				.def("graphTerm", &FramedTriplePattern::graphTerm, return_value_policy<return_by_value>())
				.def("agentTerm", &FramedTriplePattern::agentTerm, return_value_policy<return_by_value>())
				.def("beginTerm", &FramedTriplePattern::beginTerm, return_value_policy<return_by_value>())
				.def("endTerm", &FramedTriplePattern::endTerm, return_value_policy<return_by_value>())
				.def("confidenceTerm", &FramedTriplePattern::confidenceTerm, return_value_policy<return_by_value>())
				.def("objectOperator", &FramedTriplePattern::objectOperator)
				.def("isOccasionalTerm", &FramedTriplePattern::isOccasionalTerm, return_value_policy<return_by_value>())
				.def("isUncertainTerm", &FramedTriplePattern::isUncertainTerm, return_value_policy<return_by_value>())
				.def("setGraphName", &FramedTriplePattern::setGraphName);
	}
}
