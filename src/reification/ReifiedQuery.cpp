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
	static auto b_true = std::make_shared<Boolean>(true);

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
	// TODO: should we also include variables here? that would be required if context variables are allowed
	//       which is currently not really the case, I think. But if we include variables here, be careful
	//       with querying the context as it is optional! so a regular conjunctive query would not work here.
	if (nonReified.isUncertainTerm() && nonReified.isUncertainTerm().grounded()->asBoolean()) {
		create(name, reification::isUncertain, b_true, g);
	}
	if (nonReified.isOccasionalTerm() && nonReified.isOccasionalTerm().grounded()->asBoolean()) {
		create(name, reification::isOccasional, b_true, g);
	}
	if (nonReified.agentTerm().has_grounding()) {
		create(name, reification::hasPerspective, nonReified.agentTerm().grounded(), g);
	}
	if (nonReified.confidenceTerm().has_grounding()) {
		auto x = create(name, reification::hasConfidence, nonReified.confidenceTerm().grounded(), g);
		x->setObjectOperator(FramedTriplePattern::GEQ);
	}
	if (nonReified.beginTerm().has_grounding()) {
		auto x = create(name, reification::hasBeginTime, nonReified.beginTerm().grounded(), g);
		x->setObjectOperator(FramedTriplePattern::GEQ);
	}
	if (nonReified.endTerm().has_grounding()) {
		auto x = create(name, reification::hasEndTime, nonReified.endTerm().grounded(), g);
		x->setObjectOperator(FramedTriplePattern::LEQ);
	}
}

bool ReifiedQuery::isReifiable(const FramedTriplePattern &q) {
	// TODO: the variable case is a bit problematic. currently we cannot query
	//       reified and non-reified triples at the same time because we always
	//       either expand a query or not. So there should be cases where
	//       both expanded and unexpanded queries are processed in disjunction.
	//       For now, if the context has variables do not expand, meaning it is
	//       not possible to query for context variables of reified triples.
	return (q.agentTerm() && !q.agentTerm()->isVariable()) ||
		   (q.isUncertainTerm() && q.isUncertainTerm().grounded()->asBoolean()) ||
		   (q.isOccasionalTerm() && q.isOccasionalTerm().grounded()->asBoolean()) ||
		   (q.confidenceTerm() && !q.confidenceTerm()->isVariable()) ||
		   (q.beginTerm() && !q.beginTerm()->isVariable()) ||
		   (q.endTerm() && !q.endTerm()->isVariable());
}

bool ReifiedQuery::isReifiable(const std::shared_ptr<ConjunctiveQuery> &q) {
	// TODO: would the query context influence isReifiable ?
	return std::any_of(q->literals().begin(), q->literals().end(), [](const auto &x) {
		return isReifiable(*x);
	});
}
