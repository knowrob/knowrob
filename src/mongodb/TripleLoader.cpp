//
// Created by daniel on 04.04.23.
//

#include "utility"
#include "knowrob/Logger.h"
#include "knowrob/semweb/rdf.h"
#include "knowrob/semweb/rdfs.h"
#include "knowrob/semweb/owl.h"
#include "knowrob/mongodb/TripleLoader.h"
#include "knowrob/mongodb/Document.h"

using namespace knowrob::mongo;

TripleLoader::TripleLoader(std::string graphName,
                           const std::shared_ptr<Collection> &tripleCollection,
                           const std::shared_ptr<Collection> &oneCollection,
                           const semweb::VocabularyPtr &vocabulary,
                           const uint32_t batchSize)
        : graphName_(std::move(graphName)),
          tripleCollection_(tripleCollection),
          oneCollection_(oneCollection),
          batchSize_(batchSize),
          vocabulary_(vocabulary),
          operationCounter_(0),
          parentsArray_(BSON_INITIALIZER),
          timeZero_(),
          timeInfinity_()
{
    bson_decimal128_from_string ("0.0", &timeZero_);
    bson_decimal128_from_string (BSON_DECIMAL128_INF, &timeInfinity_);
}

TripleLoader::~TripleLoader()
{
    bson_destroy(&parentsArray_);
}

#define KNOWROB_BCON_NEW_TRIPLE_O1(Triple,ObjectValue) BCON_NEW(\
    "s", BCON_UTF8((Triple).subject),                 \
    "p", BCON_UTF8((Triple).predicate),               \
    "o", ObjectValue,                                 \
    "graph", BCON_UTF8(graphName.c_str()),            \
    "scope", "{", "time", "{", "since", BCON_DECIMAL128(&timeZero_), \
                               "until", BCON_DECIMAL128(&timeInfinity_), "}", "}")

#define KNOWROB_BCON_NEW_TRIPLE_O2(Triple,Parents,ObjectValue) BCON_NEW(\
    "s", BCON_UTF8((Triple).subject),                 \
    "p", BCON_UTF8((Triple).predicate),               \
    "o", ObjectValue,                                 \
    "o*", Parents,                                    \
    "graph", BCON_UTF8(graphName.c_str()),            \
    "scope", "{", "time", "{", "since", BCON_DECIMAL128(&timeZero_), \
                               "until", BCON_DECIMAL128(&timeInfinity_), "}", "}")

#define KNOWROB_BCON_NEW_TRIPLE_O3(Triple,ObjectValue) BCON_NEW(\
    "s", BCON_UTF8((Triple).subject),                 \
    "p", BCON_UTF8((Triple).predicate),               \
    "o", ObjectValue,                                 \
    "o*", "[", ObjectValue, "]",                      \
    "graph", BCON_UTF8(graphName.c_str()),            \
    "scope", "{", "time", "{", "since", BCON_DECIMAL128(&timeZero_), \
                               "until", BCON_DECIMAL128(&timeInfinity_), "}", "}")

#define KNOWROB_BCON_NEW_TRIPLE_P(Triple,Parents,ObjectValue) BCON_NEW(\
    "s", BCON_UTF8((Triple).subject),               \
    "p", BCON_UTF8((Triple).predicate),             \
    "p*", Parents,                                  \
    "o", ObjectValue,                               \
    "graph", BCON_UTF8(graphName.c_str()),          \
    "scope", "{", "time", "{", "since", BCON_DECIMAL128(&timeZero_), \
                               "until", BCON_DECIMAL128(&timeInfinity_), "}", "}")

bson_t* TripleLoader::createTripleDocument(const TripleData &tripleData,
                                           const std::string &graphName,
                                           bool isTaxonomic)
{
    auto parentsPtr = &parentsArray_;
    long counter=0;
    auto counterPtr=&counter;
    bson_reinit(&parentsArray_);

    if(isTaxonomic) {
        switch(tripleData.objectType) {
            case RDF_STRING_LITERAL:
            case RDF_RESOURCE: {
                if(vocabulary_->isDefinedProperty(tripleData.object)) {
                    // read parents array
                    vocabulary_->getDefinedProperty(tripleData.object)->forallParents(
                        [parentsPtr,counterPtr](const auto &parent){
                            auto counterKey = std::to_string((*counterPtr)++);
                            BSON_APPEND_UTF8(parentsPtr, counterKey.c_str(), parent.iri().c_str());
                        });
                    return KNOWROB_BCON_NEW_TRIPLE_O2(tripleData,
                                                      BCON_ARRAY(&parentsArray_),
                                                      BCON_UTF8(tripleData.object));
                }
                else if(vocabulary_->isDefinedClass(tripleData.object)) {
                    // read parents array
                    vocabulary_->getDefinedClass(tripleData.object)->forallParents(
                        [parentsPtr,counterPtr](const auto &parent){
                            auto counterKey = std::to_string((*counterPtr)++);
                            BSON_APPEND_UTF8(parentsPtr, counterKey.c_str(), parent.iri().c_str());
                        });
                    return KNOWROB_BCON_NEW_TRIPLE_O2(tripleData,
                                                      BCON_ARRAY(&parentsArray_),
                                                      BCON_UTF8(tripleData.object));
                }
                else {
                    return KNOWROB_BCON_NEW_TRIPLE_O3(tripleData, BCON_UTF8(tripleData.object));
                }
            }
            case RDF_DOUBLE_LITERAL:
                return KNOWROB_BCON_NEW_TRIPLE_O1(tripleData, BCON_DOUBLE(tripleData.objectDouble));
            case RDF_INT64_LITERAL:
                return KNOWROB_BCON_NEW_TRIPLE_O1(tripleData, BCON_INT64(tripleData.objectInteger));
            case RDF_BOOLEAN_LITERAL:
                return KNOWROB_BCON_NEW_TRIPLE_O1(tripleData, BCON_BOOL(tripleData.objectInteger));
        }
    }
    else {
        // read parents array
        vocabulary_->defineProperty(tripleData.predicate)->forallParents(
            [parentsPtr,counterPtr](const auto &parent){
                auto counterKey = std::to_string((*counterPtr)++);
                BSON_APPEND_UTF8(parentsPtr, counterKey.c_str(), parent.iri().c_str());
            });
        switch(tripleData.objectType) {
            case RDF_RESOURCE:
            case RDF_STRING_LITERAL: {
                return KNOWROB_BCON_NEW_TRIPLE_P(tripleData,
                                                 BCON_ARRAY(&parentsArray_),
                                                 BCON_UTF8(tripleData.object));
            }
            case RDF_DOUBLE_LITERAL:
                return KNOWROB_BCON_NEW_TRIPLE_P(tripleData,
                            BCON_ARRAY(&parentsArray_),
                            BCON_DOUBLE(tripleData.objectDouble));
            case RDF_INT64_LITERAL:
                return KNOWROB_BCON_NEW_TRIPLE_P(tripleData,
                            BCON_ARRAY(&parentsArray_),
                            BCON_INT64(tripleData.objectInteger));
            case RDF_BOOLEAN_LITERAL:
                return KNOWROB_BCON_NEW_TRIPLE_P(tripleData,
                            BCON_ARRAY(&parentsArray_),
                            BCON_BOOL(tripleData.objectInteger));
        }
    }

    return nullptr;
}

void TripleLoader::loadTriple(const TripleData &tripleData)
{
    bool isTaxonomic=false;

    // skip annotations. they may contain spacial characters and cannot be indexed.
    // TODO: optionally allow inserting annotation properties into separate collection
    if(vocabulary_->isAnnotationProperty(tripleData.predicate)) return;

    // keep track of imports, subclasses, and subproperties
    if(semweb::isSubClassOfIRI(tripleData.predicate)) {
        auto sub = vocabulary_->defineClass(tripleData.subject);
        auto sup = vocabulary_->defineClass(tripleData.object);
        sub->addDirectParent(sup);
        subClassAssertions_.emplace_back(sub, sup);
        isTaxonomic = true;
    }
    else if(semweb::isSubPropertyOfIRI(tripleData.predicate)) {
        auto sub = vocabulary_->defineProperty(tripleData.subject);
        auto sup = vocabulary_->defineProperty(tripleData.object);
        sub->addDirectParent(sup);
        subPropertyAssertions_.emplace_back(sub, sup);
        isTaxonomic = true;
    }
    else if(semweb::isTypeIRI(tripleData.predicate)) {
        isTaxonomic = true;
        vocabulary_->addResourceType(tripleData.subject, tripleData.object);
    }
    else if(semweb::isInverseOfIRI(tripleData.predicate)) {
        auto p = vocabulary_->defineProperty(tripleData.subject);
        auto q = vocabulary_->defineProperty(tripleData.object);
        p->setInverse(q);
        q->setInverse(p);
    }
    else if(semweb::IRI_imports == tripleData.predicate) {
        // TODO: maybe better for less hierarchy updates to load right away?
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

void TripleLoader::flush()
{
    if(bulkOperation_) {
        bulkOperation_->execute();
        bulkOperation_.reset();
    }
    operationCounter_ = 0;
}
