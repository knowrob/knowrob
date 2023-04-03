//
// Created by daniel on 01.04.23.
//

#include <gtest/gtest.h>
#include <utility>
#include "knowrob/Logger.h"
#include "knowrob/URI.h"
#include "knowrob/mongodb/MongoKnowledgeGraph.h"
#include "knowrob/mongodb/MongoDocument.h"
#include "knowrob/mongodb/MongoCursor.h"
#include "knowrob/mongodb/MongoInterface.h"

// aggregation pipelines
#include "pipelines/graphHierarchy.h"

#define KB_TRIPLE_VERSION_KEY "tripledbVersionString"

// TODO: better handling of IRIs
#define IRI_RDFS_SUBCLASS_OF "http://www.w3.org/2000/01/rdf-schema#subClassOf"
#define IRI_RDFS_SUBPROPERTY_OF "http://www.w3.org/2000/01/rdf-schema#subPropertyOf"
#define IRI_RDF_TYPE "http://www.w3.org/1999/02/22-rdf-syntax-ns#type"

using namespace knowrob;

MongoKnowledgeGraph::MongoKnowledgeGraph(const char* db_uri, const char* db_name, const char* collectionName)
: tripleCollection_(MongoInterface::get().connect(db_uri, db_name, collectionName))
{
    initialize();
}

MongoKnowledgeGraph::MongoKnowledgeGraph(const boost::property_tree::ptree &config)
: KnowledgeGraph()
{
    // TODO: read db_uri, db_name, collectionName from settings
    //collection_ = MongoInterface::get().connect(db_uri, db_name, collectionName);
    initialize();
}

void MongoKnowledgeGraph::initialize()
{
    createSearchIndices();
    // a collection with just a single document used for querying
    oneCollection_ = std::make_shared<MongoCollection>(
            tripleCollection_->pool(),
            tripleCollection_->client(),
            tripleCollection_->session(),
            tripleCollection_->dbName().c_str(),
            "one");
}

void MongoKnowledgeGraph::createSearchIndices()
{
    tripleCollection_->createAscendingIndex({"s"});
    tripleCollection_->createAscendingIndex({"p"});
    tripleCollection_->createAscendingIndex({"o"});
    tripleCollection_->createAscendingIndex({"p*"});
    tripleCollection_->createAscendingIndex({"o*"});
    tripleCollection_->createAscendingIndex({"s", "p"});
    tripleCollection_->createAscendingIndex({"s", "p*"});
    tripleCollection_->createAscendingIndex({"s", "o"});
    tripleCollection_->createAscendingIndex({"s", "o*"});
    tripleCollection_->createAscendingIndex({"o", "p"});
    tripleCollection_->createAscendingIndex({"o", "p*"});
    tripleCollection_->createAscendingIndex({"p", "o*"});
    tripleCollection_->createAscendingIndex({"s", "o", "p"});
    tripleCollection_->createAscendingIndex({"s", "o", "p*"});
    tripleCollection_->createAscendingIndex({"s", "o*", "p"});
}

void MongoKnowledgeGraph::drop()
{
    tripleCollection_->drop();
}

void MongoKnowledgeGraph::dropGraph(const std::string &graphName)
{
    auto document = MongoDocument(BCON_NEW("graph", BCON_UTF8(graphName.c_str())));
    tripleCollection_->removeAll(document.bson());
}

void MongoKnowledgeGraph::setCurrentGraphVersion(const std::string &graphURI,
                                                 const std::string &graphVersion,
                                                 const std::string &graphName)
{
    auto document = MongoDocument(BCON_NEW(
            "s",     BCON_UTF8(graphURI.c_str()),
            "p",     BCON_UTF8(KB_TRIPLE_VERSION_KEY),
            "o",     BCON_UTF8(graphVersion.c_str()),
            "graph", BCON_UTF8(graphName.c_str())));
    tripleCollection_->storeOne(document.bson());
}

std::optional<std::string> MongoKnowledgeGraph::getCurrentGraphVersion(const std::string &graphName)
{
    auto document = MongoDocument(BCON_NEW(
            "p",     BCON_UTF8(KB_TRIPLE_VERSION_KEY),
            "graph", BCON_UTF8(graphName.c_str())));
    const bson_t *result;
    MongoCursor cursor(tripleCollection_);
    cursor.limit(1);
    cursor.filter(document.bson());
    if(cursor.next(&result)) {
        bson_iter_t iter;
        if (bson_iter_init(&iter, result) && bson_iter_find(&iter, "o")) {
            return std::string(bson_iter_utf8(&iter, nullptr));
        }
    }
    // no version is loaded yet
    return {};
}

bool MongoKnowledgeGraph::loadTriples( //NOLINT
        const std::string &uriString, TripleFormat format)
{
    auto resolved = URI::resolve(uriString);
    auto graphName = getGraphNameFromURI(resolved);

    // check if ontology is already loaded
    auto currentVersion = getCurrentGraphVersion(graphName);
    auto newVersion = getGraphVersionFromURI(resolved);
    if(currentVersion == newVersion) {
        return true;
    }

    // delete old triples if a new version is loaded
    if(currentVersion) dropGraph(graphName);

    MongoTripleLoader loader(graphName, tripleCollection_);
    // load [s,p,o] documents into the triples collection
    if(!loadURI(loader, resolved, format)) {
        KB_WARN("failed to parse RDF file {} ({})", resolved, uriString);
        return false;
    }
    updateGraphHierarchy(loader);

    // update the version record of the ontology
    setCurrentGraphVersion(resolved, newVersion, graphName);

    // load imported ontologies
    for(auto &imported : loader.imports()) {
        loadTriples(imported, format);
    }

    return true;
}

void MongoKnowledgeGraph::updateGraphHierarchy(MongoTripleLoader &loader)
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
    for(auto &assertion : loader.subClassAssertions()) {
        auto update = MongoDocument(mngUpdateKGHierarchyO(
                tripleCollection_->name(), IRI_RDFS_SUBCLASS_OF,
                assertion.first.c_str(), assertion.second.c_str()));
        oneCollection_->evalAggregation(update.bson());
    }

    // update property hierarchy
    for(auto &assertion : loader.subPropertyAssertions()) {
        auto update = MongoDocument(mngUpdateKGHierarchyO(
                tripleCollection_->name(), IRI_RDFS_SUBPROPERTY_OF,
                assertion.first.c_str(), assertion.second.c_str()));
        oneCollection_->evalAggregation(update.bson());
    }

    // update property assertions
    std::set<std::string> visited;
    for(auto &assertion : loader.subPropertyAssertions()) {
        if(visited.count(assertion.first)>0) continue;
        visited.insert(assertion.first);

        auto update = MongoDocument(mngUpdateKGHierarchyP(
                tripleCollection_->name(), IRI_RDFS_SUBPROPERTY_OF,
                assertion.first.c_str()));
        oneCollection_->evalAggregation(update.bson());
    }
}


MongoTripleLoader::MongoTripleLoader(std::string graphName,
                                     const std::shared_ptr<MongoCollection> &collection,
                                     const uint32_t batchSize)
                                     : graphName_(std::move(graphName)),
                                       collection_(collection),
                                       batchSize_(batchSize),
                                       operationCounter_(0),
                                       timeZero_(),
                                       timeInfinity_()
{
    bson_decimal128_from_string ("0.0", &timeZero_);
    bson_decimal128_from_string (BSON_DECIMAL128_INF, &timeInfinity_);
}

static inline std::string formatBNode(const char *name, const std::string &graphName)
{
    return std::string()+name+"_"+graphName;
}

std::string MongoTripleLoader::getSubjectString(const TripleData &tripleData, const std::string &graphName)
{
    return tripleData.subjectType == RDF_BLANK ?
           formatBNode(tripleData.subject, graphName).c_str() : tripleData.subject;
}

std::string MongoTripleLoader::getObjectString(const TripleData &tripleData, const std::string &graphName)
{
    return tripleData.objectType == RDF_BLANK ?
           formatBNode(tripleData.object, graphName).c_str() : tripleData.object;
}

#define KNOWROB_BCON_SUBJECT(Triple) BCON_UTF8(Triple.subjectType == RDF_BLANK ? \
                    formatBNode(Triple.subject,graphName).c_str() : Triple.subject)

#define KNOWROB_BCON_NEW_TRIPLE_O(Triple, ObjectValue) BCON_NEW(\
    "s", KNOWROB_BCON_SUBJECT(Triple),              \
    "p", BCON_UTF8(Triple.predicate),               \
    "o", ObjectValue,                               \
    "o*", "[", ObjectValue, "]",                    \
    "graph", BCON_UTF8(graphName.c_str()),          \
    "scope", "{", "time", "{", "since", BCON_DECIMAL128(&timeZero_), \
                               "until", BCON_DECIMAL128(&timeInfinity_), "}", "}")

#define KNOWROB_BCON_NEW_TRIPLE_P(Triple, ObjectValue) BCON_NEW(\
    "s", KNOWROB_BCON_SUBJECT(Triple),              \
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
        case RDF_BLANK: {
            auto objectValue = formatBNode(tripleData.object, graphName);
            if(isTaxonomic) return KNOWROB_BCON_NEW_TRIPLE_O(tripleData, BCON_UTF8(objectValue.c_str()));
            else            return KNOWROB_BCON_NEW_TRIPLE_P(tripleData, BCON_UTF8(objectValue.c_str()));
        }
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

bool MongoTripleLoader::isAnnotationProperty(const char *propertyString)
{
    // TODO: better handling of annotation properties.
    //       best would be to maintain property hierarchy+types in memory.
    return !strcmp(propertyString, "http://www.w3.org/2000/01/rdf-schema#comment") ||
           !strcmp(propertyString, "http://www.w3.org/2000/01/rdf-schema#seeAlso") ||
           !strcmp(propertyString, "http://www.w3.org/2000/01/rdf-schema#label") ||
           !strcmp(propertyString, "http://www.w3.org/2002/07/owl#versionInfo");
}

void MongoTripleLoader::loadTriple(const TripleData &tripleData)
{
    bool isTaxonomic=false;

    // skip annotations. they may contain spacial characters and cannot ne indexed.
    // TODO: optionally allow inserting annotation properties into separate collection
    if(isAnnotationProperty(tripleData.predicate)) return;

    // keep track of imports, subclasses, and subproperties
    if(!strcmp(tripleData.predicate, "http://www.w3.org/2002/07/owl#imports")) {
        imports_.emplace_back(tripleData.object);
    }
    else if(!strcmp(tripleData.predicate, IRI_RDFS_SUBCLASS_OF)) {
        subClassAssertions_.emplace_back(getSubjectString(tripleData, graphName_),
                                         getObjectString(tripleData, graphName_));
        isTaxonomic = true;
    }
    else if(!strcmp(tripleData.predicate, IRI_RDFS_SUBPROPERTY_OF)) {
        subPropertyAssertions_.emplace_back(getSubjectString(tripleData, graphName_),
                                            getObjectString(tripleData, graphName_));
        isTaxonomic = true;
    }
    else if(!strcmp(tripleData.predicate, IRI_RDF_TYPE)) {
        isTaxonomic = true;
    }

    auto document = createTripleDocument(tripleData, graphName_, isTaxonomic);
    if(!bulkOperation_) {
        bulkOperation_ = collection_->createBulkOperation();
    }
    bulkOperation_->pushInsert(document);
    bson_free(document);

    if(operationCounter_++ > batchSize_) flush();
}

void MongoTripleLoader::flush()
{
    if(bulkOperation_) {
        bulkOperation_->execute();
        bulkOperation_.reset();
    }
    operationCounter_ = 0;
}


// fixture class for testing
class MongoKnowledgeGraphTest : public ::testing::Test {
protected:
    std::shared_ptr<MongoKnowledgeGraph> kg_;
    void SetUp() override {
        kg_ = std::make_shared<MongoKnowledgeGraph>(
                "mongodb://localhost:27017",
                "knowrob",
                "triplesTest");
        kg_->drop();
    }
    // void TearDown() override {}
};

TEST_F(MongoKnowledgeGraphTest, LoadLocal)
{
    EXPECT_FALSE(kg_->getCurrentGraphVersion("memory").has_value());
    EXPECT_NO_THROW(kg_->loadTriples("owl/test/memory.owl", knowrob::RDF_XML));
    EXPECT_TRUE(kg_->getCurrentGraphVersion("memory").has_value());
}
