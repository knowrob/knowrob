/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/Logger.h"
#include "knowrob/semweb/Vocabulary.h"
#include "knowrob/semweb/rdf.h"
#include "knowrob/semweb/rdfs.h"
#include "knowrob/semweb/owl.h"
#include "knowrob/semweb/PrefixProbe.h"
#include "knowrob/reification/model.h"

using namespace knowrob;
using namespace knowrob::semweb;

Vocabulary::Vocabulary()
	: importHierarchy_(std::make_shared<ImportHierarchy>()) {
	setPropertyFlag(rdfs::comment, ANNOTATION_PROPERTY);
	setPropertyFlag(rdfs::seeAlso, ANNOTATION_PROPERTY);
	setPropertyFlag(rdfs::label, ANNOTATION_PROPERTY);
	setPropertyFlag(owl::versionInfo, ANNOTATION_PROPERTY);

	defineProperty(rdf::type);
	defineProperty(rdfs::subPropertyOf);
	defineProperty(rdfs::subClassOf);
}

bool Vocabulary::isDefinedClass(const std::string_view &iri) {
	return definedClasses_.count(iri) > 0;
}

ClassPtr Vocabulary::getDefinedClass(const std::string_view &iri) const {
	auto it = definedClasses_.find(iri);
	if (it == definedClasses_.end()) {
		return {};
	} else {
		return it->second;
	}
}

std::vector<ClassPtr> Vocabulary::getDefinedClassesWithPrefix(const std::string_view &prefix) const {
	auto range_it = definedClasses_.equal_range(PrefixProbe{prefix});
	std::vector<ClassPtr> result;
	for (auto it = range_it.first; it != range_it.second; ++it) {
		result.push_back(it->second);
	}
	return result;
}

std::vector<std::string_view> Vocabulary::getDefinedClassNamesWithPrefix(const std::string_view &prefix) const {
	auto range_it = definedClasses_.equal_range(PrefixProbe{prefix});
	std::vector<std::string_view> result;
	for (auto it = range_it.first; it != range_it.second; ++it) {
		result.push_back(it->first);
	}
	return result;
}

ClassPtr Vocabulary::defineClass(const std::string_view &iri) {
	auto it = definedClasses_.find(iri);
	if (it == definedClasses_.end()) {
		auto newClass = std::make_shared<Class>(iri);
		definedClasses_[newClass->iri()] = newClass;
		return newClass;
	} else {
		return it->second;
	}
}

void Vocabulary::addSubClassOf(const std::string_view &subClass, const std::string_view &superClass) {
	defineClass(subClass)->addDirectParent(defineClass(superClass));
}

bool Vocabulary::isSubClassOf(const std::string_view &subClass, const std::string_view &superClass) {
	return defineClass(subClass)->isSubClassOf(defineClass(superClass));
}

bool Vocabulary::isDefinedProperty(const std::string_view &iri) {
	return definedProperties_.count(iri) > 0;
}

bool Vocabulary::isDefinedReification(const std::string_view &iri) {
	return definedReifications_.count(iri) > 0;
}

PropertyPtr Vocabulary::getDefinedProperty(const std::string_view &iri) const {
	auto it = definedProperties_.find(iri);
	if (it == definedProperties_.end()) {
		return {};
	} else {
		return it->second;
	}
}

PropertyPtr Vocabulary::getDefinedReification(const std::string_view &iri) const {
	auto it = definedReifications_.find(iri);
	if (it == definedReifications_.end()) {
		return {};
	} else {
		return it->second;
	}
}

std::vector<PropertyPtr> Vocabulary::getDefinedPropertiesWithPrefix(const std::string_view &prefix) const {
	auto range_it = definedProperties_.equal_range(PrefixProbe{prefix});
	std::vector<PropertyPtr> result;
	for (auto it = range_it.first; it != range_it.second; ++it) {
		result.push_back(it->second);
	}
	return result;
}

std::vector<std::string_view> Vocabulary::getDefinedPropertyNamesWithPrefix(const std::string_view &prefix) const {
	auto range_it = definedProperties_.equal_range(PrefixProbe{prefix});
	std::vector<std::string_view> result;
	for (auto it = range_it.first; it != range_it.second; ++it) {
		result.push_back(it->first);
	}
	return result;
}

PropertyPtr Vocabulary::defineProperty(const std::string_view &iri) {
	auto it = definedProperties_.find(iri);
	if (it == definedProperties_.end()) {
		return defineProperty(std::make_shared<Property>(iri));
	} else {
		return it->second;
	}
}

PropertyPtr Vocabulary::defineProperty(const IRIAtomPtr &iri) {
	auto it = definedProperties_.find(iri->stringForm());
	if (it == definedProperties_.end()) {
		return defineProperty(std::make_shared<Property>(iri));
	} else {
		return it->second;
	}
}

PropertyPtr Vocabulary::defineProperty(const std::shared_ptr<Property> &p) {
	definedProperties_[p->iri()] = p;
	definedReifications_[p->reification()->iri()] = p;
	// define the reification of the property as a concept which inherits knowrob:ReifiedRelation
	// note further superclasses will be added through Property::addDirectParent
	auto reification = p->reification();
	definedClasses_[reification->iri()] = reification;
	reification->addDirectParent(defineClass(reification::ReifiedRelation->stringForm()));
	return p;
}

void Vocabulary::setPropertyFlag(const std::string_view &iri, PropertyFlag flag) {
	defineProperty(iri)->setFlag(flag);
}

void Vocabulary::setPropertyFlag(const IRIAtomPtr &iri, PropertyFlag flag) {
	defineProperty(iri)->setFlag(flag);
}

void Vocabulary::addSubPropertyOf(const std::string_view &subProperty, const std::string_view &superProperty) {
	defineProperty(subProperty)->addDirectParent(defineProperty(superProperty));
}

void Vocabulary::setInverseOf(const std::string_view &a, const std::string_view &b) {
	auto a1 = defineProperty(a);
	auto b1 = defineProperty(b);
	a1->setInverse(b1);
	b1->setInverse(a1);
}

bool Vocabulary::isAnnotationProperty(const std::string_view &iri) {
	auto it = definedProperties_.find(iri);
	return it != definedProperties_.end() && it->second->hasFlag(ANNOTATION_PROPERTY);
}

bool Vocabulary::isObjectProperty(const std::string_view &iri) {
	auto it = definedProperties_.find(iri);
	return it != definedProperties_.end() && it->second->hasFlag(OBJECT_PROPERTY);
}

bool Vocabulary::isDatatypeProperty(const std::string_view &iri) {
	auto it = definedProperties_.find(iri);
	return it != definedProperties_.end() && it->second->hasFlag(DATATYPE_PROPERTY);
}

bool Vocabulary::isTaxonomicProperty(const std::string_view &iri) {
	return isSubClassOfIRI(iri) ||
		   isSubPropertyOfIRI(iri) ||
		   isTypeIRI(iri);
}

// FIXME: the frequency of parent/child properties and classes should be managed too!

void Vocabulary::setFrequency(const std::string_view &iri, uint64_t frequency) {
	if (frequency == 0) {
		frequency_.erase(iri);
	} else {
		auto p_it = definedProperties_.find(iri);
		if (p_it != definedProperties_.end()) {
			frequency_[p_it->second->iri()] = frequency;
			return;
		}
		auto c_it = definedClasses_.find(iri);
		if (c_it != definedClasses_.end()) {
			frequency_[c_it->second->iri()] = frequency;
			return;
		}
		KB_WARN("cannot set frequency for unknown IRI '{}'", iri);
	}
}

void Vocabulary::increaseFrequency(const std::string_view &iri) {
	auto it = frequency_.find(iri);
	if (it == frequency_.end()) {
		setFrequency(iri, 1);
	} else {
		it->second++;
	}
}

uint64_t Vocabulary::frequency(const std::string_view &iri) const {
	auto it = frequency_.find(iri);
	if (it == frequency_.end()) return 0;
	else return it->second;
}


void Vocabulary::addResourceType(const std::string_view &resource_iri, const std::string_view &type_iri) {
	if (isObjectPropertyIRI(type_iri))
		setPropertyFlag(resource_iri, OBJECT_PROPERTY);
	else if (isDatatypePropertyIRI(type_iri))
		setPropertyFlag(resource_iri, DATATYPE_PROPERTY);
	else if (isAnnotationPropertyIRI(type_iri))
		setPropertyFlag(resource_iri, ANNOTATION_PROPERTY);
	else if (isReflexivePropertyIRI(type_iri))
		setPropertyFlag(resource_iri, REFLEXIVE_PROPERTY);
	else if (isTransitivePropertyIRI(type_iri))
		setPropertyFlag(resource_iri, TRANSITIVE_PROPERTY);
	else if (isSymmetricPropertyIRI(type_iri))
		setPropertyFlag(resource_iri, SYMMETRIC_PROPERTY);
	else if (isPropertyIRI(type_iri))
		defineProperty(resource_iri);
	else if (isClassIRI(type_iri))
		defineClass(resource_iri);
}
