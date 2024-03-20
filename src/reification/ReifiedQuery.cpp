/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <utility>

#include "knowrob/reification/ReifiedQuery.h"
#include "knowrob/semweb/rdf.h"
#include "knowrob/reification/model.h"
#include "knowrob/semweb/Property.h"
#include "knowrob/queries/QueryError.h"

using namespace knowrob;

ReifiedQuery::ReifiedQuery(const std::shared_ptr<GraphQuery> &nonReified, VocabularyPtr vocabulary, bool withFullFrame)
		: GraphQuery(nonReified->ctx()),
		  vocabulary_(std::move(vocabulary)),
		  withFullFrame_(withFullFrame),
		  varCounter_(0) {
	setNonReified(nonReified->term());
}

ReifiedQuery::ReifiedQuery(const FramedTriplePattern &nonReified, VocabularyPtr vocabulary, bool withFullFrame)
		: GraphQuery(),
		  vocabulary_(std::move(vocabulary)),
		  withFullFrame_(withFullFrame),
		  varCounter_(0) {
	term_ = reifiedPatternSequence(nonReified);
}

void ReifiedQuery::setNonReified(const std::shared_ptr<GraphTerm> &nonReified) {
	switch (nonReified->termType()) {
		case GraphTermType::Pattern:
			term_ = reifyPattern(std::static_pointer_cast<GraphPattern>(nonReified));
			break;
		case GraphTermType::Union:
			term_ = reifyUnion(std::static_pointer_cast<GraphUnion>(nonReified));
			break;
		case GraphTermType::Sequence:
			term_ = reifySequence(std::static_pointer_cast<GraphSequence>(nonReified));
			break;
		case GraphTermType::Builtin:
			term_ = nonReified;
			break;
	}
}

void ReifiedQuery::reifyConnective(const std::shared_ptr<GraphConnective> &reifiedConnective, //NOLINT
								   const std::shared_ptr<GraphConnective> &originalConnective) {
	for (auto &t: originalConnective->terms()) {
		switch (t->termType()) {
			case GraphTermType::Pattern:
				reifiedConnective->addMember(reifyPattern(std::static_pointer_cast<GraphPattern>(t)));
				break;
			case GraphTermType::Union:
				reifiedConnective->addMember(reifyUnion(std::static_pointer_cast<GraphUnion>(t)));
				break;
			case GraphTermType::Sequence:
				reifiedConnective->addMember(reifySequence(std::static_pointer_cast<GraphSequence>(t)));
				break;
			case GraphTermType::Builtin:
				reifiedConnective->addMember(t);
				break;
		}
	}
}

std::shared_ptr<GraphUnion> ReifiedQuery::reifyUnion(const std::shared_ptr<GraphUnion> &graphUnion) { //NOLINT
	auto reified = std::make_shared<GraphUnion>();
	reifyConnective(reified, graphUnion);
	return reified;
}

std::shared_ptr<GraphSequence>
ReifiedQuery::reifySequence(const std::shared_ptr<GraphSequence> &graphSequence) { //NOLINT
	auto reified = std::make_shared<GraphSequence>();
	reifyConnective(reified, graphSequence);
	return reified;
}

std::shared_ptr<GraphTerm> ReifiedQuery::reifyPattern(const std::shared_ptr<GraphPattern> &nonReified) {
	auto flags = getReificationFlags(*nonReified->value());
	if (flags & IncludeReified && flags & IncludeOriginal) {
		// if both reified and original are included, we need to create a union of both
		auto graphUnion = std::make_shared<GraphUnion>();
		graphUnion->addMember(nonReified);
		graphUnion->addMember(reifiedPatternSequence(*nonReified->value()));
		return graphUnion;
	} else if (flags & IncludeReified) {
		return reifiedPatternSequence(*nonReified->value());
	} else {
		return nonReified;
	}
}

std::shared_ptr<FramedTriplePattern> addPattern(
		const std::shared_ptr<GraphSequence> &sequence,
		const TermPtr &s,
		const TermPtr &p,
		const TermPtr &o,
		const groundable<Atom> &g) {
	auto reified = std::make_shared<FramedTriplePattern>(s, p, o);
	if (g.has_grounding()) {
		reified->setGraphName(g.grounded()->stringForm());
	}
	sequence->addPattern(reified);
	return reified;
}

std::shared_ptr<GraphTerm> ReifiedQuery::reifiedPatternSequence(const FramedTriplePattern &nonReified) {
	static auto fullyConfident = std::make_shared<Double>(1.0);
	static auto egoPerspective = Perspective::getEgoPerspective()->atom();
	static auto b_var = std::make_shared<Variable>("_reified_b");
	static auto e_var = std::make_shared<Variable>("_reified_e");

	if (!nonReified.propertyTerm() || !nonReified.propertyTerm()->isAtom()) {
		throw QueryError("non-reified triple does not have a property term");
	}
	auto seq = std::make_shared<GraphSequence>();
	auto p_atom = std::static_pointer_cast<Atom>(nonReified.propertyTerm());
	auto property = vocabulary_->defineProperty(p_atom->stringForm());
	// map the property to a Relation concept
	auto relationType = property->reification();
	// generate a unique individual variable.
	// note that we are actually not interested in its instantiation.
	auto name = std::make_shared<Variable>("_reified" + std::to_string(varCounter_++));
	// optionally set origin of reified queries if term is grounded
	auto &g = nonReified.graphTerm();

	// create a type assertion
	addPattern(seq, name, rdf::type, relationType->iriAtom(), g);

	// create a query for subject
	addPattern(seq, name, reification::hasSubject, nonReified.subjectTerm(), g);

	// create a query for the object/literal value
	std::shared_ptr<FramedTriplePattern> objectQuery;
	if (property->isObjectProperty()) {
		objectQuery = addPattern(seq, name, reification::hasObject, nonReified.objectTerm(), g);
	} else {
		objectQuery = addPattern(seq, name, reification::hasLiteral, nonReified.objectTerm(), g);
	}
	objectQuery->setObjectOperator(nonReified.objectOperator());

	// create queries for optional properties
	bool includeOnlyCertain;
	if (nonReified.isUncertainTerm().has_grounding() && nonReified.isUncertainTerm().grounded()->asBoolean()) {
		if (withFullFrame_) {
			auto x = addPattern(seq, name, reification::isUncertain, Numeric::trueAtom(), g);
			x->setIsOptional(true);
		}
		includeOnlyCertain = false;
	} else if (nonReified.isUncertainTerm().has_variable()) {
		auto x = addPattern(seq, name, reification::isUncertain, nonReified.isUncertainTerm().variable(), g);
		x->setIsOptional(true);
		includeOnlyCertain = false;
	} else {
		auto x = addPattern(seq, name, reification::isUncertain, Numeric::falseAtom(), g);
		x->setIsOptional(true);
		includeOnlyCertain = true;
	}

	if (nonReified.confidenceTerm().has_grounding()) {
		auto x = addPattern(seq, name, reification::hasConfidence, nonReified.confidenceTerm().grounded(), g);
		x->setObjectOperator(FilterType::GEQ);
		x->setIsOptional(true);
	} else if (nonReified.confidenceTerm().has_variable()) {
		auto x = addPattern(seq, name, reification::hasConfidence, nonReified.confidenceTerm().variable(), g);
		x->setIsOptional(true);
	} else if (includeOnlyCertain) {
		auto x = addPattern(seq, name, reification::hasConfidence, fullyConfident, g);
		x->setObjectOperator(FilterType::GEQ);
		x->setIsOptional(true);
	}

	if (nonReified.perspectiveTerm().has_grounding()) {
		if (Perspective::isEgoPerspective(nonReified.perspectiveTerm().grounded()->stringForm())) {
			auto x = addPattern(seq, name, reification::hasPerspective, egoPerspective, g);
			x->setIsOptional(true);
		} else {
			addPattern(seq, name, reification::hasPerspective, nonReified.perspectiveTerm().grounded(), g);
		}
	} else if (nonReified.perspectiveTerm().has_variable()) {
		auto x = addPattern(seq, name, reification::hasPerspective, nonReified.perspectiveTerm().variable(), g);
		x->setIsOptional(true);
	} else {
		auto x = addPattern(seq, name, reification::hasPerspective, egoPerspective, g);
		x->setIsOptional(true);
	}

	bool includeOccasional;
	if (nonReified.isOccasionalTerm().has_grounding() && nonReified.isOccasionalTerm().grounded()->asBoolean()) {
		if (withFullFrame_) {
			auto x = addPattern(seq, name, reification::isOccasional, Numeric::trueAtom(), g);
			x->setIsOptional(true);
		}
		includeOccasional = true;
	} else if (nonReified.isOccasionalTerm().has_variable()) {
		auto x = addPattern(seq, name, reification::isOccasional, nonReified.isOccasionalTerm().variable(), g);
		x->setIsOptional(true);
		includeOccasional = true;
	} else {
		auto x = addPattern(seq, name, reification::isOccasional, Numeric::falseAtom(), g);
		x->setIsOptional(true);
		includeOccasional = false;
	}

	if (nonReified.beginTerm().has_grounding()) {
		if (includeOccasional) {
			auto x = addPattern(seq, name, reification::hasEndTime, nonReified.beginTerm().grounded(), g);
			x->setObjectOperator(FilterType::GEQ);
			x->setIsOptional(true);
		} else {
			auto x = addPattern(seq, name, reification::hasBeginTime, nonReified.beginTerm().grounded(), g);
			x->setObjectOperator(FilterType::LEQ);
			x->setIsOptional(true);
		}
	} else if (nonReified.beginTerm().has_variable()) {
		auto x = addPattern(seq, name, reification::hasBeginTime, nonReified.beginTerm().variable(), g);
		x->setIsOptional(true);
	} else {
		auto x = addPattern(seq, name, reification::hasBeginTime, b_var, g);
		x->setIsNegated(true);
	}

	if (nonReified.endTerm().has_grounding()) {
		if (includeOccasional) {
			auto x = addPattern(seq, name, reification::hasBeginTime, nonReified.endTerm().grounded(), g);
			x->setObjectOperator(FilterType::LEQ);
			x->setIsOptional(true);
		} else {
			auto x = addPattern(seq, name, reification::hasEndTime, nonReified.endTerm().grounded(), g);
			x->setObjectOperator(FilterType::GEQ);
			x->setIsOptional(true);
		}
	} else if (nonReified.endTerm().has_variable()) {
		auto x = addPattern(seq, name, reification::hasEndTime, nonReified.endTerm().variable(), g);
		x->setIsOptional(true);
	} else {
		auto x = addPattern(seq, name, reification::hasEndTime, e_var, g);
		x->setIsNegated(true);
	}

	return seq;
}

bool ReifiedQuery::hasReifiablePattern(const GraphTerm *term) { //NOLINT
	switch (term->termType()) {
		case GraphTermType::Pattern:
			return getReificationFlags(*((const GraphPattern *) term)->value()) & IncludeReified;
		case GraphTermType::Union:
			for (auto &t: ((const GraphUnion *) term)->terms()) {
				if (hasReifiablePattern(t.get())) {
					return true;
				}
			}
			return false;
		case GraphTermType::Sequence:
			for (auto &t: ((const GraphSequence *) term)->terms()) {
				if (hasReifiablePattern(t.get())) {
					return true;
				}
			}
			return false;
		case GraphTermType::Builtin:
			return false;
	}
	return false;
}

bool ReifiedQuery::hasReifiablePattern(const std::shared_ptr<GraphQuery> &nonReified) {
	return hasReifiablePattern(nonReified->term().get());
}

int ReifiedQuery::getReificationFlags(const FramedTriplePattern &q) {
	bool includeOriginal = true;
	bool includeReified = false;
	// include reified if isUncertain is true or a variable in the query
	if ((q.isUncertainTerm().has_grounding() && q.isUncertainTerm().grounded()->asBoolean()) ||
		q.isUncertainTerm().has_variable()) {
		includeReified = true;
	}
	// include reified if confidence has a value or is a variable
	if (q.confidenceTerm().has_grounding() || q.confidenceTerm().has_variable()) {
		includeReified = true;
	}
	// include reified if perspective is not the ego perspective or a variable.
	// In case perspective is not ego perspective, the original triples are not included.
	if (q.perspectiveTerm().has_grounding()) {
		auto perspective = q.perspectiveTerm().grounded();
		if (!Perspective::isEgoPerspective(perspective->stringForm())) {
			includeOriginal = false;
			includeReified = true;
		}
	} else if (q.perspectiveTerm().has_variable()) {
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
