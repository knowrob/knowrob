/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/reification/ReifiedTriple.h"
#include "knowrob/reification/model.h"
#include "knowrob/semweb/rdf.h"
#include "knowrob/semweb/Property.h"

using namespace knowrob;

ReifiedTriple::ReifiedTriple(const FramedTriple &triple, const semweb::VocabularyPtr &vocabulary,
							 const IRIAtomPtr &reifiedName) {
	auto property = vocabulary->defineProperty(triple.predicate());
	// map the property to a Relation concept
	auto relationType = property->reification();
	// generate a unique individual name
	name_ = reifiedName ? reifiedName : semweb::Resource::unique_iri(
			reification::individualPrefix->stringForm(),
			semweb::Resource::iri_name(relationType->iri()));
	const auto &name = name_->stringForm();
	// set origin of the reified triples
	auto g = triple.graph();

	// need to avoid copy of FramedTriplePtr, so we reserve space in the vector to fit them exactly
	uint32_t triple_count = 3;
	if (triple.perspective()) triple_count += 1;
	if (triple.isUncertain() || triple.confidence()) triple_count += 1;
	if (triple.isOccasional()) triple_count += 1;
	if (triple.confidence()) triple_count += 1;
	if (triple.begin()) triple_count += 1;
	if (triple.end()) triple_count += 1;
	reified_.reserve(triple_count);

	// create a rdf:type assertion for the relation individual
	create(name, semweb::rdf::type, g)->setObjectIRI(relationType->iri());

	// set the subject, and object of the reified triple
	if (triple.isSubjectBlank()) {
		create(name, reification::hasSubject, g)->setObjectBlank(triple.subject());
	} else {
		create(name, reification::hasSubject, g)->setObjectIRI(triple.subject());
	}
	if (triple.isObjectBlank()) {
		create(name, reification::hasObject, g)->setObjectBlank(triple.valueAsString());
	} else if (triple.isObjectIRI()) {
		create(name, reification::hasObject, g)->setObjectIRI(triple.valueAsString());
	} else if (triple.xsdType()) {
		if (triple.xsdType().value() == XSDType::STRING) {
			create(name, reification::hasLiteral, g)->setXSDValue(triple.valueAsString(), XSDType::STRING);
		} else {
			generatedString_ = triple.createStringValue();
			create(name, reification::hasLiteral, g)->setXSDValue(generatedString_, triple.xsdType().value());
		}
	}

	// set the optional properties of the reified triple
	if (triple.perspective()) {
		create(name, reification::hasPerspective, g)->setObjectIRI(triple.perspective().value());
	}
	if (triple.isUncertain() || triple.confidence()) {
		create(name, reification::isUncertain, g)->setBooleanValue(true);
	}
	if (triple.isOccasional()) {
		create(name, reification::isOccasional, g)->setBooleanValue(true);
	}
	if (triple.confidence()) {
		create(name, reification::hasConfidence, g)->setDoubleValue(triple.confidence().value());
	}
	if (triple.begin()) {
		create(name, reification::hasBeginTime, g)->setDoubleValue(triple.begin().value());
	}
	if (triple.end()) {
		create(name, reification::hasEndTime, g)->setDoubleValue(triple.end().value());
	}
}

FramedTriple *
ReifiedTriple::create(std::string_view subject, const AtomPtr &property, const std::optional<std::string_view> &g) {
	auto &reified = reified_.emplace_back(new FramedTripleView());
	reified->setSubject(subject);
	reified->setPredicate(property->stringForm());
	if (g.has_value()) {
		reified->setGraph(g.value());
	}
	return reified.ptr;
}

bool ReifiedTriple::isPartOfReification(const FramedTriple &triple) {
	return triple.subject().compare(
			0,
			reification::individualPrefix->stringForm().size(),
			reification::individualPrefix->stringForm()) == 0;
}

bool ReifiedTriple::isReifiable(const FramedTriple &triple) {
	bool hasNonEgoPerspective = triple.perspective() && !Perspective::isEgoPerspective(triple.perspective().value());
	return hasNonEgoPerspective ||
		   triple.isUncertain() ||
		   triple.isOccasional() ||
		   triple.confidence() ||
		   triple.begin() ||
		   triple.end();
}
