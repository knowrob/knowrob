//
// Created by daniel on 04.04.23.
//

#include "utility"
#include "sstream"
#include "knowrob/graphs/rdf.h"
#include "knowrob/graphs/rdfs.h"
#include "knowrob/graphs/owl.h"
#include "knowrob/mongodb/MongoTripleLoader.h"
#include "knowrob/mongodb/MongoDocument.h"
#include "pipelines/graphHierarchy.h"

using namespace knowrob;

std::set<std::string_view> MongoTripleLoader::annotationProperties_ = std::set<std::string_view>();
bool MongoTripleLoader::hasStaticInitialization_ = false;

MongoTripleLoader::MongoTripleLoader(std::string graphName,
                                     const std::shared_ptr<MongoCollection> &tripleCollection,
                                     const std::shared_ptr<MongoCollection> &oneCollection,
                                     const uint32_t batchSize)
        : graphName_(std::move(graphName)),
          tripleCollection_(tripleCollection),
          oneCollection_(oneCollection),
          batchSize_(batchSize),
          operationCounter_(0),
          timeZero_(),
          timeInfinity_()
{
    bson_decimal128_from_string ("0.0", &timeZero_);
    bson_decimal128_from_string (BSON_DECIMAL128_INF, &timeInfinity_);
    if(!hasStaticInitialization_) {
        hasStaticInitialization_ = true;
        annotationProperties_.insert(rdfs::IRI_comment);
        annotationProperties_.insert(rdfs::IRI_seeAlso);
        annotationProperties_.insert(rdfs::IRI_label);
        annotationProperties_.insert(owl::IRI_versionInfo);
    }
}

#define KNOWROB_BCON_NEW_TRIPLE_O(Triple,ObjectValue) BCON_NEW(\
    "s", BCON_UTF8(Triple.subject),                 \
    "p", BCON_UTF8(Triple.predicate),               \
    "o", ObjectValue,                               \
    "o*", "[", ObjectValue, "]",                    \
    "graph", BCON_UTF8(graphName.c_str()),          \
    "scope", "{", "time", "{", "since", BCON_DECIMAL128(&timeZero_), \
                               "until", BCON_DECIMAL128(&timeInfinity_), "}", "}")

#define KNOWROB_BCON_NEW_TRIPLE_P(Triple,ObjectValue) BCON_NEW(\
    "s", BCON_UTF8(Triple.subject),                 \
    "p", BCON_UTF8(Triple.predicate),               \
    "o", ObjectValue,                               \
    "p*", "[", BCON_UTF8(Triple.predicate), "]",    \
    "graph", BCON_UTF8(graphName.c_str()),          \
    "scope", "{", "time", "{", "since", BCON_DECIMAL128(&timeZero_), \
                               "until", BCON_DECIMAL128(&timeInfinity_), "}", "}")

bson_t* MongoTripleLoader::createTripleDocument(const TripleData &tripleData,
                                                const std::string &graphName,
                                                bool isTaxonomic)
{
    switch(tripleData.objectType) {
        case RDF_RESOURCE:
        case RDF_STRING_LITERAL:
            if(isTaxonomic) return KNOWROB_BCON_NEW_TRIPLE_O(tripleData, BCON_UTF8(tripleData.object));
            else            return KNOWROB_BCON_NEW_TRIPLE_P(tripleData, BCON_UTF8(tripleData.object));
        case RDF_DOUBLE_LITERAL: {
            auto objectValue = strtod(tripleData.object, nullptr);
            if(isTaxonomic) return KNOWROB_BCON_NEW_TRIPLE_O(tripleData, BCON_DOUBLE(objectValue));
            else            return KNOWROB_BCON_NEW_TRIPLE_P(tripleData, BCON_DOUBLE(objectValue));
        }
        case RDF_INT64_LITERAL: {
            auto objectValue = strtol(tripleData.object, nullptr, 10);
            if(isTaxonomic) return KNOWROB_BCON_NEW_TRIPLE_O(tripleData, BCON_INT64(objectValue));
            else            return KNOWROB_BCON_NEW_TRIPLE_P(tripleData, BCON_INT64(objectValue));
        }
        case RDF_BOOLEAN_LITERAL: {
            bool objectValue;
            std::istringstream(tripleData.object) >> objectValue;
            if(isTaxonomic) return KNOWROB_BCON_NEW_TRIPLE_O(tripleData, BCON_BOOL(objectValue));
            else            return KNOWROB_BCON_NEW_TRIPLE_P(tripleData, BCON_BOOL(objectValue));
        }
    }

    return nullptr;
}

void MongoTripleLoader::loadTriple(const TripleData &tripleData)
{
    bool isTaxonomic=false;

    // skip annotations. they may contain spacial characters and cannot be indexed.
    // TODO: optionally allow inserting annotation properties into separate collection
    if(annotationProperties_.count(
            std::string_view(tripleData.predicate)) > 0) return;

    // keep track of imports, subclasses, and subproperties
    if(rdfs::isSubClassOfIRI(tripleData.predicate)) {
        subClassAssertions_.emplace_back(tripleData.subject, tripleData.object);
        isTaxonomic = true;
    }
    else if(rdfs::isSubPropertyOfIRI(tripleData.predicate)) {
        subPropertyAssertions_.emplace_back(tripleData.subject, tripleData.object);
        isTaxonomic = true;
    }
    else if(rdf::isTypeIRI(tripleData.predicate)) {
        isTaxonomic = true;
    }
    else if(owl::IRI_imports == tripleData.predicate) {
        imports_.emplace_back(tripleData.object);
    }

    auto document = createTripleDocument(tripleData, graphName_, isTaxonomic);
    if(!bulkOperation_) {
        bulkOperation_ = tripleCollection_->createBulkOperation();
    }
    bulkOperation_->pushInsert(document);
    bson_free(document);

    if(operationCounter_++ > batchSize_) flush();
}

void MongoTripleLoader::finish()
{
    // below performs the update purely server-side.
    // each step computes the parents in class or property hierarchy.
    // However, there are many steps for large ontologies so this is very
    // time consuming.
    // TODO: can the hierarchy update be done faster?
    //       - keeping the class and property hierarchy in a std::map could speed things up.
    //       - try doing all updates in one call.
    //         But it seems problematic doing all updates in one call using $merge stage. seems
    //         iterations cannot access updates made in previous iterations.

    // update class hierarchy
    for(auto &assertion : subClassAssertions_) {
        MongoDocument update(mngUpdateKGHierarchyO(
                tripleCollection_->name(), rdfs::IRI_subClassOf.c_str(),
                assertion.first, assertion.second));
        oneCollection_->evalAggregation(update);
    }

    // update property hierarchy
    std::set<std::string_view> visited;
    for(auto &assertion : subPropertyAssertions_) {
        visited.insert(assertion.first);
        MongoDocument update(mngUpdateKGHierarchyO(
                tripleCollection_->name(), rdfs::IRI_subPropertyOf.c_str(),
                assertion.first, assertion.second));
        oneCollection_->evalAggregation(update);
    }

    // update property assertions
    for(auto &newProperty : visited) {
        MongoDocument update(mngUpdateKGHierarchyP(
                tripleCollection_->name(), rdfs::IRI_subPropertyOf.c_str(),
                newProperty.data()));
        oneCollection_->evalAggregation(update);
    }
}

void MongoTripleLoader::flush()
{
    if(bulkOperation_) {
        bulkOperation_->execute();
        bulkOperation_.reset();
    }
    operationCounter_ = 0;
}
