/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <utility>

#include "knowrob/reification/ReifiedQuery.h"
#include "knowrob/semweb/rdf.h"
#include "knowrob/reification/model.h"
#include "knowrob/semweb/Property.h"

using namespace knowrob;

ReifiedQuery::ReifiedQuery(const std::shared_ptr<ConjunctiveQuery> &nonReified, semweb::VocabularyPtr vocabulary)
		: ConjunctiveQuery(nonReified->ctx()),
		  vocabulary_(std::move(vocabulary)) {
	for (const auto &pat: nonReified->literals()) {
		addNonReified(*pat);
	}
	init();
}

ReifiedQuery::ReifiedQuery(const FramedTriplePattern &nonReified, semweb::VocabularyPtr vocabulary)
		: ConjunctiveQuery(),
		  vocabulary_(std::move(vocabulary)) {
	addNonReified(nonReified);
	init();
}

std::shared_ptr<FramedTriplePattern> ReifiedQuery::create(const TermPtr &s, const TermPtr &p, const TermPtr &o, const groundable<Atom> &g) {
	auto reified = std::make_shared<FramedTriplePattern>(s, p, o);
	if(g.has_grounding()) {
		reified->setGraphName(g.grounded()->stringForm());
	}
	literals_.push_back(reified);
	return reified;
}

void ReifiedQuery::addNonReified(const FramedTriplePattern &nonReified) {
	static auto b_true = Numeric::trueAtom();
	static auto b_false = Numeric::falseAtom();
	static auto b_var = std::make_shared<Variable>("v_reified_b");
	static auto e_var = std::make_shared<Variable>("v_reified_e");
	static auto fullyConfident = std::make_shared<Double>(1.0);
	static auto egoPerspective = Perspective::getEgoPerspective()->atom();

	if (!nonReified.propertyTerm() || !nonReified.propertyTerm()->isAtom()) {
		KB_WARN("non-reified triple does not have a property term, ignoring");
		return;
	}
	auto p_atom = std::static_pointer_cast<Atom>(nonReified.propertyTerm());
	auto property = vocabulary_->defineProperty(p_atom->stringForm());
	// map the property to a Relation concept
	auto relationType = property->reification();
	// generate a unique individual name
	auto name = semweb::Resource::unique_iri(
			reification::individualPrefix->stringForm(),
			semweb::Resource::iri_name(relationType->iri()));
	// optionally set origin of reified queries if term is grounded
	auto g = nonReified.graphTerm();

	// create a type assertion
	create(name, semweb::rdf::type, relationType->iriAtom(), g);

	// create a query for subject
	create(name, reification::hasSubject, nonReified.subjectTerm(), g);

	// create a query for the object/literal value
	std::shared_ptr<FramedTriplePattern> objectQuery;
	if (property->isObjectProperty()) {
		objectQuery = create(name, reification::hasObject, nonReified.objectTerm(), g);
	} else {
		objectQuery = create(name, reification::hasLiteral, nonReified.objectTerm(), g);
	}
	objectQuery->setObjectOperator(nonReified.objectOperator());

	// create queries for optional properties
	bool includeOnlyCertain;
	if (nonReified.isUncertainTerm().has_grounding() && nonReified.isUncertainTerm().grounded()->asBoolean()) {
		create(name, reification::isUncertain, b_true, g);
		includeOnlyCertain = false;
	} else if (nonReified.isUncertainTerm().has_variable()) {
		auto x = create(name, reification::isUncertain, nonReified.isUncertainTerm().variable(), g);
		x->setIsOptional(true);
		includeOnlyCertain = false;
	} else {
		auto x = create(name, reification::isUncertain, b_false, g);
		x->setIsOptional(true);
		includeOnlyCertain = true;
	}

	if (nonReified.confidenceTerm().has_grounding()) {
		auto x = create(name, reification::hasConfidence, nonReified.confidenceTerm().grounded(), g);
		x->setObjectOperator(FramedTriplePattern::GEQ);
	} else if (nonReified.confidenceTerm().has_variable()) {
		auto x = create(name, reification::hasConfidence, nonReified.confidenceTerm().variable(), g);
		x->setIsOptional(true);
	} else if (includeOnlyCertain) {
		auto x = create(name, reification::hasConfidence, fullyConfident, g);
		x->setObjectOperator(FramedTriplePattern::GEQ);
		x->setIsOptional(true);
	}

	if (nonReified.agentTerm().has_grounding()) {
		if (Perspective::isEgoPerspective(nonReified.agentTerm().grounded()->stringForm())) {
			auto x = create(name, reification::hasPerspective, egoPerspective, g);
			x->setIsOptional(true);
		} else {
			create(name, reification::hasPerspective, nonReified.agentTerm().grounded(), g);
		}
	} else if (nonReified.agentTerm().has_variable()) {
		auto x = create(name, reification::hasPerspective, nonReified.agentTerm().variable(), g);
		x->setIsOptional(true);
	} else {
		auto x = create(name, reification::hasPerspective, egoPerspective, g);
		x->setIsOptional(true);
	}

	bool includeOccasional;
	if (nonReified.isOccasionalTerm().has_grounding() && nonReified.isOccasionalTerm().grounded()->asBoolean()) {
		create(name, reification::isOccasional, b_true, g);
		includeOccasional = true;
	} else if (nonReified.isOccasionalTerm().has_variable()) {
		auto x = create(name, reification::isOccasional, nonReified.isOccasionalTerm().variable(), g);
		x->setIsOptional(true);
		includeOccasional = true;
	} else {
		auto x = create(name, reification::isOccasional, b_false, g);
		x->setIsOptional(true);
		includeOccasional = false;
	}

	if (nonReified.beginTerm().has_grounding()) {
		if (includeOccasional) {
			auto x = create(name, reification::hasEndTime, nonReified.beginTerm().grounded(), g);
			x->setObjectOperator(FramedTriplePattern::GEQ);
			x->setIsOptional(true);
		} else {
			auto x = create(name, reification::hasBeginTime, nonReified.beginTerm().grounded(), g);
			x->setObjectOperator(FramedTriplePattern::LEQ);
			x->setIsOptional(true);
		}
	} else if (nonReified.beginTerm().has_variable()) {
		auto x = create(name, reification::hasBeginTime, nonReified.beginTerm().variable(), g);
		x->setIsOptional(true);
	} else {
		auto x = create(name, reification::hasBeginTime, b_var, g);
		x->setIsNegated(true);
	}

	if (nonReified.endTerm().has_grounding()) {
		if (includeOccasional) {
			auto x = create(name, reification::hasBeginTime, nonReified.endTerm().grounded(), g);
			x->setObjectOperator(FramedTriplePattern::LEQ);
			x->setIsOptional(true);
		} else {
			auto x = create(name, reification::hasEndTime, nonReified.endTerm().grounded(), g);
			x->setObjectOperator(FramedTriplePattern::GEQ);
			x->setIsOptional(true);
		}
	} else if (nonReified.endTerm().has_variable()) {
		auto x = create(name, reification::hasEndTime, nonReified.endTerm().variable(), g);
		x->setIsOptional(true);
	} else {
		auto x = create(name, reification::hasEndTime, e_var, g);
		x->setIsNegated(true);
	}
}

int ReifiedQuery::getReificationFlags(const FramedTriplePattern &q) {
	bool includeOriginal = true;
	bool includeReified = false;
	// include reified if isUncertain is true or a variable in the query
	if ((q.isUncertainTerm().has_grounding() && q.isUncertainTerm().grounded()->asBoolean()) || q.isUncertainTerm().has_variable()) {
		includeReified = true;
	}
	// include reified if confidence has a value or is a variable
	if (q.confidenceTerm().has_grounding() || q.confidenceTerm().has_variable()) {
		includeReified = true;
	}
	// include reified if perspective is not the ego perspective or a variable.
	// In case perspective is not ego perspective, the original triples are not included.
	if (q.agentTerm().has_grounding()) {
		auto perspective = q.agentTerm().grounded();
		if (!Perspective::isEgoPerspective(perspective->stringForm())) {
			includeOriginal = false;
			includeReified = true;
		}
	} else if (q.agentTerm().has_variable()) {
		includeReified = true;
	}
	// include reified if isOccasional is true or a variable in the query.
	// In case isOccasional is true, the original triples are not included.
	if (q.isOccasionalTerm().has_grounding()) {
		auto occasional = q.isOccasionalTerm().grounded();
		if (occasional->asBoolean()) {
			includeReified = true;
			includeOriginal = false;
		}
	} else if (q.isOccasionalTerm().has_variable()) {
		includeReified = true;
	}
	// include reified if begin or end is a variable or has a grounding.
	if (q.beginTerm().has_grounding() || q.beginTerm().has_variable() ||
	    q.endTerm().has_grounding() || q.endTerm().has_variable()) {
		includeReified = true;
	}
	int flags = 0;
	if (includeOriginal) flags |= IncludeOriginal;
	if (includeReified) flags |= IncludeReified;
	return flags;
}
