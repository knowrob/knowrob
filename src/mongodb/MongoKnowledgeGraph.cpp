//
// Created by daniel on 01.04.23.
//

#include <gtest/gtest.h>
#include <filesystem>
#include "knowrob/Logger.h"
#include "knowrob/URI.h"
#include "knowrob/mongodb/MongoKnowledgeGraph.h"
#include "knowrob/mongodb/MongoDocument.h"
#include "knowrob/mongodb/MongoCursor.h"
#include "knowrob/mongodb/MongoInterface.h"
#include "knowrob/mongodb/MongoTripleLoader.h"

#define MONGO_KG_ONE_COLLECTION "one"
#define MONGO_KG_VERSION_KEY "tripledbVersionString"

#define MONGO_KG_SETTING_HOST "host"
#define MONGO_KG_SETTING_PORT "port"
#define MONGO_KG_SETTING_USER "user"
#define MONGO_KG_SETTING_PASSWORD "password"
#define MONGO_KG_SETTING_DB "db"
#define MONGO_KG_SETTING_COLLECTION "collection"

#define MONGO_KG_DEFAULT_HOST "localhost"
#define MONGO_KG_DEFAULT_PORT "27017"
#define MONGO_KG_DEFAULT_DB "knowrob"
#define MONGO_KG_DEFAULT_COLLECTION "triples"

using namespace knowrob;

MongoKnowledgeGraph::MongoKnowledgeGraph(const char* db_uri, const char* db_name, const char* collectionName)
: KnowledgeGraph(),
  tripleCollection_(MongoInterface::get().connect(db_uri, db_name, collectionName))
{
    initialize();
}

MongoKnowledgeGraph::MongoKnowledgeGraph(const boost::property_tree::ptree &config)
: KnowledgeGraph(),
  tripleCollection_(MongoInterface::get().connect(
          getURI(config).c_str(),
          getDBName(config),
          getCollectionName(config)))
{
    initialize();
}

const char* MongoKnowledgeGraph::getDBName(const boost::property_tree::ptree &config)
{
    static const char *defaultDBName = MONGO_KG_DEFAULT_DB;
    auto o_dbname = config.get_optional<std::string>(MONGO_KG_SETTING_DB);
    return (o_dbname ? o_dbname->c_str() : defaultDBName);
}

const char* MongoKnowledgeGraph::getCollectionName(const boost::property_tree::ptree &config)
{
    static const char *defaultCollectionName = MONGO_KG_DEFAULT_COLLECTION;
    auto o_collection = config.get_optional<std::string>(MONGO_KG_SETTING_COLLECTION);
    return (o_collection ? o_collection->c_str() : defaultCollectionName);
}

std::string MongoKnowledgeGraph::getURI(const boost::property_tree::ptree &config)
{
    auto o_host = config.get_optional<std::string>(MONGO_KG_SETTING_HOST);
    auto o_port = config.get_optional<std::string>(MONGO_KG_SETTING_PORT);
    auto o_user = config.get_optional<std::string>(MONGO_KG_SETTING_USER);
    auto o_password = config.get_optional<std::string>(MONGO_KG_SETTING_PASSWORD);
    // format URI of the form "mongodb://USER:PW@HOST:PORT"
    std::stringstream uriStream;
    uriStream << "mongodb://";
    if(o_user) {
        uriStream << o_user.value();
        if(o_password) uriStream << ':' << o_password.value();
        uriStream << '@';
    }
    uriStream
        << (o_host ? o_host.value() : MONGO_KG_DEFAULT_HOST)
        << ':'
        << (o_port ? o_port.value() : MONGO_KG_DEFAULT_PORT);
    return uriStream.str();
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
            MONGO_KG_ONE_COLLECTION);
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

void MongoKnowledgeGraph::setCurrentGraphVersion(const std::string &graphName,
                                                 const std::string &graphURI,
                                                 const std::string &graphVersion)
{
    auto document = MongoDocument(BCON_NEW(
            "s",     BCON_UTF8(graphURI.c_str()),
            "p",     BCON_UTF8(MONGO_KG_VERSION_KEY),
            "o",     BCON_UTF8(graphVersion.c_str()),
            "graph", BCON_UTF8(graphName.c_str())));
    tripleCollection_->storeOne(document.bson());
}

std::optional<std::string> MongoKnowledgeGraph::getCurrentGraphVersion(const std::string &graphName)
{
    auto document = MongoDocument(BCON_NEW(
            "p",     BCON_UTF8(MONGO_KG_VERSION_KEY),
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
    if(currentVersion) {
        // ontology was loaded before
        if(currentVersion == newVersion) return true;
        // delete old triples if a new version is loaded
        dropGraph(graphName);
    }

    MongoTripleLoader loader(graphName, tripleCollection_, oneCollection_);
    // some OWL files are downloaded compile-time via CMake,
    // they are downloaded into owl/external e.g. there are SOMA.owl and DUL.owl.
    // TODO: rework handling of cmake-downloaded ontologies, e.g. should also work when installed
    auto p =  std::filesystem::path(KNOWROB_SOURCE_DIR) / "owl" / "external" /
        std::filesystem::path(resolved).filename();
    const std::string *importURI = (exists(p) ? &p.native() : &resolved);

    // define a prefix for naming blank nodes
    std::string blankPrefix("_");
    blankPrefix += graphName;

    KB_INFO("Loading ontology at '{}' with version "
            "\"{}\" into graph \"{}\".", *importURI, newVersion, graphName);
    // load [s,p,o] documents into the triples collection
    if(!loadURI(loader, *importURI, blankPrefix, format)) {
        KB_WARN("Failed to parse ontology {} ({})", *importURI, uriString);
        return false;
    }
    // update the version record of the ontology
    setCurrentGraphVersion(graphName, resolved, newVersion);
    // load imported ontologies
    for(auto &imported : loader.imports()) loadTriples(imported, format);

    return true;
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
