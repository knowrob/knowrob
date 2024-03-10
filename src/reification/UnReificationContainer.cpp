/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/reification/UnReificationContainer.h"
#include "knowrob/reification/model.h"
#include "knowrob/semweb/rdf.h"
#include "knowrob/semweb/Property.h"

using namespace knowrob;

semweb::TripleContainer::ConstGenerator UnReificationContainer::cgenerator() const {
	return [this, it = triples_.begin()]() mutable -> const FramedTriplePtr * {
		if (it == triples_.end()) return nullptr;
		auto &triple = it->second;
		++it;
		return &triple;
	};
}

FramedTriplePtr &UnReificationContainer::getUnReifiedTriple(std::string_view subject) {
	auto it = triples_.find(subject);
	if (it == triples_.end()) {
		auto [jt, success] = triples_.emplace(subject, new FramedTripleView());
		return jt->second;
	}
	return it->second;
}

void UnReificationContainer::add(const FramedTriple &triple) {
	using PropertyHandler = std::function<void(const FramedTriple &triple, FramedTriplePtr &unReified)>;
	static std::map<std::string_view, PropertyHandler> propertyHandler = {
			{semweb::rdf::type->stringForm(),           [](auto &triple, auto &unReified) {
				unReified->setPredicate(semweb::Property::unReifiedIRI(triple.valueAsString())->stringForm());
			}},
			{reification::hasSubject->stringForm(),     [](auto &triple, auto &unReified) {
				if (rdfNodeTypeGuess(triple.valueAsString()) == RDFNodeType::BLANK) {
					unReified->setSubjectBlank(triple.valueAsString());
				} else {
					unReified->setSubject(triple.valueAsString());
				}
			}},
			{reification::hasObject->stringForm(),      [](auto &triple, auto &unReified) {
				if (rdfNodeTypeGuess(triple.valueAsString()) == RDFNodeType::BLANK) {
					unReified->setObjectBlank(triple.valueAsString());
				} else {
					unReified->setObjectIRI(triple.valueAsString());
				}
			}},
			{reification::hasLiteral->stringForm(),     [](auto &triple, auto &unReified) {
				unReified->setXSDValue(triple.valueAsString(), triple.xsdType().value());
			}},
			{reification::hasPerspective->stringForm(), [](auto &triple, auto &unReified) {
				unReified->setPerspective(triple.valueAsString());
			}},
			{reification::isUncertain->stringForm(),    [](auto &triple, auto &unReified) {
				unReified->setIsUncertain(triple.valueAsBoolean());
			}},
			{reification::isOccasional->stringForm(),   [](auto &triple, auto &unReified) {
				unReified->setIsOccasional(triple.valueAsBoolean());
			}},
			{reification::hasConfidence->stringForm(),  [](auto &triple, auto &unReified) {
				unReified->setConfidence(triple.valueAsDouble());
			}},
			{reification::hasBeginTime->stringForm(),   [](auto &triple, auto &unReified) {
				unReified->setBegin(triple.valueAsDouble());
			}},
			{reification::hasEndTime->stringForm(),     [](auto &triple, auto &unReified) {
				unReified->setEnd(triple.valueAsDouble());
			}}
	};

	auto it = propertyHandler.find(triple.predicate());
	if (it == propertyHandler.end()) {
		KB_WARN("No property handler for {}.", triple.predicate());
		return;
	}
	auto &handler = it->second;
	handler(triple, getUnReifiedTriple(triple.subject()));
}
