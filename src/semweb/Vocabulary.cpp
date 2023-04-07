//
// Created by daniel on 07.04.23.
//

#include "knowrob/Logger.h"
#include "knowrob/semweb/Vocabulary.h"
#include "knowrob/semweb/rdf.h"
#include "knowrob/semweb/rdfs.h"
#include "knowrob/semweb/owl.h"

using namespace knowrob::semweb;

Vocabulary::Vocabulary()
{
    setPropertyFlag(IRI_comment, ANNOTATION_PROPERTY);
    setPropertyFlag(IRI_seeAlso, ANNOTATION_PROPERTY);
    setPropertyFlag(IRI_label, ANNOTATION_PROPERTY);
    setPropertyFlag(IRI_versionInfo, ANNOTATION_PROPERTY);

    auto it = definedProperties_.find(IRI_comment);
    if(it == definedProperties_.end()) {
        KB_WARN("not a defined property");
    }
    else if(!it->second->hasFlag(ANNOTATION_PROPERTY)) {
        KB_WARN("not a annotation property");
    }
}

bool Vocabulary::isDefinedClass(std::string_view iri)
{
    return definedClasses_.count(iri)>0;
}

ClassPtr Vocabulary::getDefinedClass(std::string_view iri) const
{
    auto it = definedClasses_.find(iri);
    if(it == definedClasses_.end()) {
        return {};
    }
    else {
        return it->second;
    }
}

ClassPtr Vocabulary::defineClass(std::string_view iri)
{
    auto it = definedClasses_.find(iri);
    if(it == definedClasses_.end()) {
        auto newClass = std::make_shared<Class>(iri);
        definedClasses_[newClass->iri()] = newClass;
        return newClass;
    }
    else {
        return it->second;
    }
}

void Vocabulary::addSubClassOf(std::string_view subClass, std::string_view superClass)
{
    defineClass(subClass)->addDirectParent(defineClass(superClass));
}


bool Vocabulary::isDefinedProperty(std::string_view iri)
{
    return definedProperties_.count(iri)>0;
}

PropertyPtr Vocabulary::getDefinedProperty(std::string_view iri) const
{
    auto it = definedProperties_.find(iri);
    if(it == definedProperties_.end()) {
        return {};
    }
    else {
        return it->second;
    }
}

PropertyPtr Vocabulary::defineProperty(std::string_view iri)
{
    auto it = definedProperties_.find(iri);
    if(it == definedProperties_.end()) {
        auto newProperty = std::make_shared<Property>(iri);
        definedProperties_[newProperty->iri()] = newProperty;
        return newProperty;
    }
    else {
        return it->second;
    }
}

void Vocabulary::setPropertyFlag(std::string_view iri, PropertyFlag flag)
{
    defineProperty(iri)->setFlag(flag);
}

void Vocabulary::addSubPropertyOf(std::string_view subProperty, std::string_view superProperty)
{
    defineProperty(subProperty)->addDirectParent(defineProperty(superProperty));
}

bool Vocabulary::isAnnotationProperty(std::string_view iri)
{
    auto it = definedProperties_.find(iri);
    return it != definedProperties_.end() && it->second->hasFlag(ANNOTATION_PROPERTY);
}


void Vocabulary::addResourceType(std::string_view resource_iri, std::string_view type_iri)
{
    if(isObjectPropertyIRI(type_iri))
        setPropertyFlag(resource_iri, OBJECT_PROPERTY);
    else if(isDatatypePropertyIRI(type_iri))
        setPropertyFlag(resource_iri, DATATYPE_PROPERTY);
    else if(isAnnotationPropertyIRI(type_iri))
        setPropertyFlag(resource_iri, ANNOTATION_PROPERTY);
    else if(isReflexivePropertyIRI(type_iri))
        setPropertyFlag(resource_iri, REFLEXIVE_PROPERTY);
    else if(isTransitivePropertyIRI(type_iri))
        setPropertyFlag(resource_iri, TRANSITIVE_PROPERTY);
    else if(isSymmetricPropertyIRI(type_iri))
        setPropertyFlag(resource_iri, SYMMETRIC_PROPERTY);
    else if(isPropertyIRI(type_iri))
        defineProperty(resource_iri);
    else if(isClassIRI(type_iri))
        defineClass(resource_iri);
}
