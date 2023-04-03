//
// Created by daniel on 01.04.23.
//

#include <gtest/gtest.h>
#include <utility>
#include <filesystem>
#include "knowrob/Logger.h"
#include "knowrob/URI.h"
#include "knowrob/graphs/rdf.h"
#include "knowrob/graphs/rdfs.h"
#include "knowrob/graphs/owl.h"
#include "knowrob/mongodb/MongoKnowledgeGraph.h"
#include "knowrob/mongodb/MongoDocument.h"
#include "knowrob/mongodb/MongoCursor.h"
#include "knowrob/mongodb/MongoInterface.h"
#include "pipelines/graphHierarchy.h"

#define KB_TRIPLE_VERSION_KEY "tripledbVersionString"

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

    MongoTripleLoader loader(graphName, tripleCollection_, oneCollection_);
    // some OWL files are downloaded compile-time via CMake,
    // they are downloaded into owl/external e.g. there are SOMA.owl and DUL.owl.
    // TODO: rework handling of cmake-downloaded ontologies, e.g. should also work when installed
    auto p =  std::filesystem::path(KNOWROB_SOURCE_DIR) / "owl" / "external" /
        std::filesystem::path(resolved).filename();
    const std::string *importURI = (exists(p) ? &p.native() : &resolved);

    KB_INFO("Loading ontology at '{}' with version \"{}\" into graph \"{}\".", *importURI, newVersion, graphName);
    // load [s,p,o] documents into the triples collection
    if(!loadURI(loader, *importURI, graphName, format)) {
        KB_WARN("Failed to parse ontology {} ({})", *importURI, uriString);
        return false;
    }

    // update the version record of the ontology
    setCurrentGraphVersion(resolved, newVersion, graphName);

    // load imported ontologies
    for(auto &imported : loader.imports()) {
        loadTriples(imported, format);
    }

    return true;
}

std::set<std::string_view> MongoTripleLoader::annotationProperties_ = std::set<std::string_view>();
bool MongoTripleLoader::hasStaticInitialization_ = false;

MongoTripleLoader::MongoTripleLoader(std::string graphName,
                                     const std::shared_ptr<MongoCollection> &tripleCollection,
                                     const std::shared_ptr<MongoCollection> &oneCollection,
                                     const uint32_t batchSize)
                                     : graphName_(std::move(graphName)),
                                       blankPrefix_(std::string("_")+graphName),
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

    // skip annotations. they may contain spacial characters and cannot ne indexed.
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
        auto update = MongoDocument(mngUpdateKGHierarchyO(
                tripleCollection_->name(), rdfs::IRI_subClassOf.c_str(),
                assertion.first, assertion.second));
        oneCollection_->evalAggregation(update.bson());
    }

    // update property hierarchy
    for(auto &assertion : subPropertyAssertions_) {
        auto update = MongoDocument(mngUpdateKGHierarchyO(
                tripleCollection_->name(), rdfs::IRI_subPropertyOf.c_str(),
                assertion.first, assertion.second));
        oneCollection_->evalAggregation(update.bson());
    }

    // update property assertions
    std::set<std::string_view> visited;
    for(auto &assertion : subPropertyAssertions_) {
        if(visited.count(assertion.first)>0) continue;
        visited.insert(assertion.first);

        auto update = MongoDocument(mngUpdateKGHierarchyP(
                tripleCollection_->name(), rdfs::IRI_subPropertyOf.c_str(),
                assertion.first));
        oneCollection_->evalAggregation(update.bson());
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
