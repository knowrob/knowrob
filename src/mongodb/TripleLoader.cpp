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
using namespace knowrob::semweb;

TripleLoader::TripleLoader(std::string graphName,
                           const std::shared_ptr<Collection> &tripleCollection,
                           const std::shared_ptr<Collection> &oneCollection,
                           const VocabularyPtr &vocabulary,
                           const uint32_t batchSize)
        : graphName_(std::move(graphName)),
          tripleCollection_(tripleCollection),
          oneCollection_(oneCollection),
          batchSize_(batchSize),
          vocabulary_(vocabulary),
          operationCounter_(0)
{
}

TripleLoader::~TripleLoader()
= default;

bson_t* TripleLoader::createTripleDocument(const StatementData &tripleData,
                                           const std::string &graphName,
                                           bool isTaxonomic)
{
    bson_t parentsArray;
    uint32_t arrIndex=0;
    auto counterPtr=&arrIndex;

    bson_t *tripleDoc = bson_new();
    BSON_APPEND_UTF8(tripleDoc, "s", tripleData.subject);
    BSON_APPEND_UTF8(tripleDoc, "p", tripleData.predicate);

    if(isTaxonomic) {
        switch(tripleData.objectType) {
            case RDF_STRING_LITERAL:
            case RDF_RESOURCE: {
                BSON_APPEND_UTF8(tripleDoc, "o", tripleData.object);

                BSON_APPEND_ARRAY_BEGIN(tripleDoc, "o*", &parentsArray);
                auto parentsPtr = &parentsArray;
                if(vocabulary_->isDefinedProperty(tripleData.object)) {
                    vocabulary_->getDefinedProperty(tripleData.object)->forallParents(
                        [parentsPtr,counterPtr](const auto &parent){
                            auto counterKey = std::to_string((*counterPtr)++);
                            BSON_APPEND_UTF8(parentsPtr, counterKey.c_str(), parent.iri().c_str());
                        });
                }
                else if(vocabulary_->isDefinedClass(tripleData.object)) {
                    // read parents array
                    vocabulary_->getDefinedClass(tripleData.object)->forallParents(
                        [parentsPtr,counterPtr](const auto &parent){
                            auto counterKey = std::to_string((*counterPtr)++);
                            BSON_APPEND_UTF8(parentsPtr, counterKey.c_str(), parent.iri().c_str());
                        });
                }
                else {
                    BSON_APPEND_UTF8(&parentsArray, "0", tripleData.object);
                }
                bson_append_array_end(tripleDoc, &parentsArray);
                break;
            }
            case RDF_DOUBLE_LITERAL:
                BSON_APPEND_DOUBLE(tripleDoc, "o", tripleData.objectDouble);
                break;
            case RDF_INT64_LITERAL:
                BSON_APPEND_INT64(tripleDoc, "o", tripleData.objectInteger);
                break;
            case RDF_BOOLEAN_LITERAL:
                BSON_APPEND_BOOL(tripleDoc, "o", tripleData.objectInteger);
                break;
        }
    }
    else {
        switch(tripleData.objectType) {
            case RDF_RESOURCE:
            case RDF_STRING_LITERAL:
                BSON_APPEND_UTF8(tripleDoc, "o", tripleData.object);
                break;
            case RDF_DOUBLE_LITERAL:
                BSON_APPEND_DOUBLE(tripleDoc, "o", tripleData.objectDouble);
                break;
            case RDF_INT64_LITERAL:
                BSON_APPEND_INT64(tripleDoc, "o", tripleData.objectInteger);
                break;
            case RDF_BOOLEAN_LITERAL:
                BSON_APPEND_BOOL(tripleDoc, "o", tripleData.objectInteger);
                break;
        }
        // read parents array
        BSON_APPEND_ARRAY_BEGIN(tripleDoc, "p*", &parentsArray);
        auto parentsPtr = &parentsArray;
        vocabulary_->defineProperty(tripleData.predicate)->forallParents(
            [parentsPtr,counterPtr](const auto &parent){
                auto counterKey = std::to_string((*counterPtr)++);
                BSON_APPEND_UTF8(parentsPtr, counterKey.c_str(), parent.iri().c_str());
            });
        bson_append_array_end(tripleDoc, &parentsArray);
    }

    BSON_APPEND_UTF8(tripleDoc, "graph", graphName.c_str());
    if(tripleData.agent)
        BSON_APPEND_UTF8(tripleDoc, "agent", tripleData.agent);

    bool isBelief = false;
    if(tripleData.confidence.has_value()) {
        BSON_APPEND_DOUBLE(tripleDoc, "confidence", tripleData.confidence.value());
        isBelief = true;
    }
    else if(tripleData.epistemicOperator.has_value()) {
        isBelief = (tripleData.epistemicOperator.value() == EpistemicOperator::BELIEF);
    }
    if(isBelief) {
        // flag the statement as "uncertain"
        BSON_APPEND_BOOL(tripleDoc, "uncertain", true);
    }

    if(tripleData.temporalOperator.has_value() && tripleData.temporalOperator.value() == TemporalOperator::SOMETIMES) {
        // flag the statement as "occasional", meaning it is only known that it was true at some past instants
        BSON_APPEND_BOOL(tripleDoc, "occasional", true);
    }

    if(tripleData.begin.has_value() || tripleData.end.has_value()) {
        bson_t scopeDoc, timeDoc;
        BSON_APPEND_DOCUMENT_BEGIN(tripleDoc, "scope", &scopeDoc);
        BSON_APPEND_DOCUMENT_BEGIN(&scopeDoc, "time", &timeDoc);
        if(tripleData.begin.has_value()) BSON_APPEND_DOUBLE(&timeDoc, "since", tripleData.begin.value());
        if(tripleData.end.has_value())   BSON_APPEND_DOUBLE(&timeDoc, "until", tripleData.end.value());
        bson_append_document_end(&scopeDoc, &timeDoc);
        bson_append_document_end(tripleDoc, &scopeDoc);
    }

    return tripleDoc;
}

void TripleLoader::loadTriple(const StatementData &tripleData)
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
    else if(owl::imports == tripleData.predicate) {
        // TODO: maybe better for less hierarchy updates to load right away?
        imports_.emplace_back(tripleData.object);
    }
    else if(semweb::isPropertyIRI(tripleData.predicate)) {
    	// increase assertion counter which is used in ordering metrics
    	vocabulary_->increaseFrequency(tripleData.predicate);
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
