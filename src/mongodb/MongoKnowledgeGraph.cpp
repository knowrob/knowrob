//
// Created by daniel on 01.04.23.
//

#include <gtest/gtest.h>
#include <filesystem>
#include "knowrob/Logger.h"
#include "knowrob/URI.h"
#include "knowrob/mongodb/MongoKnowledgeGraph.h"
#include "knowrob/mongodb/Document.h"
#include "knowrob/mongodb/Cursor.h"
#include "knowrob/mongodb/MongoInterface.h"
#include "knowrob/mongodb/TripleCursor.h"
#include "knowrob/mongodb/aggregation/graph.h"
#include "knowrob/mongodb/aggregation/triples.h"
#include "knowrob/semweb/rdf.h"
#include "knowrob/semweb/rdfs.h"
#include "knowrob/semweb/owl.h"
#include "knowrob/terms/ListTerm.h"

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
using namespace knowrob::mongo;

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
    oneCollection_ = std::make_shared<Collection>(
            tripleCollection_->pool(),
            tripleCollection_->client(),
            tripleCollection_->session(),
            tripleCollection_->dbName().c_str(),
            MONGO_KG_ONE_COLLECTION);

    // initialize vocabulary
    TripleData tripleData;
    {
        // iterate over all rdf::type assertions
        // TODO: could avoid going over all type assertions by looking for specific type iris
        TripleCursor cursor(tripleCollection_);
        cursor.filter(Document(BCON_NEW(
            "p", BCON_UTF8(semweb::IRI_type.c_str()))).bson());
        while(cursor.nextTriple(tripleData))
            vocabulary_->addResourceType(tripleData.subject, tripleData.object);
    }
    {
        // iterate over all rdfs::subClassOf assertions
        TripleCursor cursor(tripleCollection_);
        cursor.filter(Document(BCON_NEW(
            "p", BCON_UTF8(semweb::IRI_subClassOf.c_str()))).bson());
        while(cursor.nextTriple(tripleData))
            vocabulary_->addSubClassOf(tripleData.subject, tripleData.object);
    }
    {
        // iterate over all rdfs::subPropertyOf assertions
        TripleCursor cursor(tripleCollection_);
        cursor.filter(Document(BCON_NEW(
            "p", BCON_UTF8(semweb::IRI_subPropertyOf.c_str()))).bson());
        while(cursor.nextTriple(tripleData))
            vocabulary_->addSubPropertyOf(tripleData.subject, tripleData.object);
    }
    {
        // iterate over all owl::inverseOf assertions
        TripleCursor cursor(tripleCollection_);
        cursor.filter(Document(BCON_NEW(
            "p", BCON_UTF8(semweb::IRI_inverseOf.c_str()))).bson());
        while(cursor.nextTriple(tripleData))
            vocabulary_->setInverseOf(tripleData.subject, tripleData.object);
    }
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
    vocabulary_ = std::make_shared<semweb::Vocabulary>();
}

void MongoKnowledgeGraph::dropGraph(const std::string &graphName)
{
    tripleCollection_->removeAll(Document(
            BCON_NEW("graph", BCON_UTF8(graphName.c_str()))));
}

void MongoKnowledgeGraph::setCurrentGraphVersion(const std::string &graphName,
                                                 const std::string &graphURI,
                                                 const std::string &graphVersion)
{
    tripleCollection_->storeOne(Document(BCON_NEW(
            "s",     BCON_UTF8(graphURI.c_str()),
            "p",     BCON_UTF8(MONGO_KG_VERSION_KEY),
            "o",     BCON_UTF8(graphVersion.c_str()),
            "graph", BCON_UTF8(graphName.c_str()))));
}

std::optional<std::string> MongoKnowledgeGraph::getCurrentGraphVersion(const std::string &graphName)
{
    auto document = Document(BCON_NEW(
            "p",     BCON_UTF8(MONGO_KG_VERSION_KEY),
            "graph", BCON_UTF8(graphName.c_str())));
    const bson_t *result;
    Cursor cursor(tripleCollection_);
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

bson_t* MongoKnowledgeGraph::getTripleSelector(
            const semweb::TripleExpression &tripleExpression,
            bool b_isTaxonomicProperty)
{
    auto doc = bson_new();
    aggregation::appendTripleSelector(doc, tripleExpression, b_isTaxonomicProperty);
    return doc;
}

void MongoKnowledgeGraph::assertTriple(const TripleData &tripleData)
{
    TripleLoader loader("user",
                        tripleCollection_,
                        oneCollection_,
                        vocabulary_);
    loader.loadTriple(tripleData);
    loader.flush();
    updateHierarchy(loader);
}

void MongoKnowledgeGraph::assertTriples(const std::vector<TripleData> &tripleData)
{
    TripleLoader loader("user",
                        tripleCollection_,
                        oneCollection_,
                        vocabulary_);
    auto loaderPtr = &loader;
    std::for_each(tripleData.begin(), tripleData.end(),
        [loaderPtr](auto &data) {
            loaderPtr->loadTriple(data);
        });
    loader.flush();
    updateHierarchy(loader);
}

void MongoKnowledgeGraph::removeAllTriples(const semweb::TripleExpression &tripleExpression)
{
    bool b_isTaxonomicProperty = isTaxonomicProperty(tripleExpression.propertyTerm());
    tripleCollection_->removeAll(
        Document(getTripleSelector(tripleExpression, b_isTaxonomicProperty)));
}

void MongoKnowledgeGraph::removeOneTriple(const semweb::TripleExpression &tripleExpression)
{
    bool b_isTaxonomicProperty = isTaxonomicProperty(tripleExpression.propertyTerm());
    tripleCollection_->removeOne(
        Document(getTripleSelector(tripleExpression, b_isTaxonomicProperty)));
}

std::shared_ptr<AnswerCursor> MongoKnowledgeGraph::lookupTriples(const semweb::TripleExpression &tripleExpression)
{
    bson_t pipelineDoc = BSON_INITIALIZER;
    bson_t pipelineArray;

    BSON_APPEND_ARRAY_BEGIN(&pipelineDoc, "pipeline", &pipelineArray);
    aggregation::Pipeline pipeline(&pipelineArray); {
        // append lookup stages to pipeline
        aggregation::TripleLookupData lookupData(&tripleExpression);
        // indicate that no variables in tripleExpression may have been instantiated
        // by a previous step to allow for some optimizations.
        lookupData.mayHasMoreGroundings = false;
        aggregation::lookupTriple(pipeline, tripleCollection_->name(), vocabulary_, lookupData);
    }
    bson_append_array_end(&pipelineDoc, &pipelineArray);

    auto cursor = std::make_shared<AnswerCursor>(oneCollection_);
    cursor->aggregate(&pipelineDoc);
    return cursor;
    //return std::make_shared<Cursor>(oneCollection_, &pipelineDoc, true);
}

BufferedAnswerStreamPtr MongoKnowledgeGraph::submitQuery(const GraphQueryPtr &query)
{
    // TODO: masterplan:
    //  - re-implement triple querying in c++, this might take some hours!
    //  - use a worker thread to pull from the cursor and push into stream,
    //    but also allow synchronous processing via a flag
    //
    // TODO: what mechanism is used to convert modality labels to query parameters
    //       and selection of collections etc.?
    //
    return {};
}

BufferedAnswerStreamPtr MongoKnowledgeGraph::watchQuery(const GraphQueryPtr &literal)
{
    // TODO
    return {};
}

bool MongoKnowledgeGraph::loadTriples( //NOLINT
        const std::string &uriString, TripleFormat format)
{
    // TODO: rather format IRI's as e.g. "soma:foo" and store namespaces somehow as part of graph?
    //          or just assume namespace prefixes are unique withing knowrob
    auto resolved = URI::resolve(uriString);
    auto graphName = getNameFromURI(resolved);

    // check if ontology is already loaded
    auto currentVersion = getCurrentGraphVersion(graphName);
    auto newVersion = getVersionFromURI(resolved);
    if(currentVersion) {
        // ontology was loaded before
        if(currentVersion == newVersion) return true;
        // delete old triples if a new version is loaded
        dropGraph(graphName);
    }

    TripleLoader loader(graphName, tripleCollection_, oneCollection_, vocabulary_);
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
    // update o* and p* fields
    updateHierarchy(loader);
    // load imported ontologies
    for(auto &imported : loader.imports()) loadTriples(imported, format);

    return true;
}

void MongoKnowledgeGraph::updateHierarchy(TripleLoader &tripleLoader)
{
    // below performs the server-side data transformation for updating hierarchy relations
    // such as rdf::type.
    // However, there are many steps for large ontologies so this might consume some time.
    // TODO: list of parents could be supplied as a constant in aggregation queries below.
    //       currently parents are computed in the query, maybe it would be a bit faster using a constant
    //       baked into the query.
    // TODO: the same pipeline template is used many times below.
    //       is there some means one could optimize this? e.g. by generating only once a bson_t
    //       and then modifying it? not sure if it would be worth it though.

    bson_t pipelineDoc = BSON_INITIALIZER;

    // update class hierarchy.
    // unfortunately must be done step-by-step as it is undefined yet in mongo
    // if it's possible to access $merge results in following pipeline iterations
    // via e.g. $lookup.
    for(auto &assertion : tripleLoader.subClassAssertions()) {
        bson_reinit(&pipelineDoc);

        bson_t pipelineArray;
        BSON_APPEND_ARRAY_BEGIN(&pipelineDoc, "pipeline", &pipelineArray);
        aggregation::Pipeline pipeline(&pipelineArray);
        aggregation::updateHierarchyO(pipeline,
                        tripleCollection_->name(),
                        semweb::IRI_subClassOf,
                        assertion.first->iri(),
                        assertion.second->iri());
        bson_append_array_end(&pipelineDoc, &pipelineArray);

        oneCollection_->evalAggregation(&pipelineDoc);
    }

    // update property hierarchy.
    // unfortunately must be done step-by-step as it is undefined yet in mongo
    // if it's possible to access $merge results in following pipeline iterations
    // via e.g. $lookup.
    std::set<std::string_view> visited;
    for(auto &assertion : tripleLoader.subPropertyAssertions()) {
        visited.insert(assertion.first->iri());
        bson_reinit(&pipelineDoc);

        bson_t pipelineArray;
        BSON_APPEND_ARRAY_BEGIN(&pipelineDoc, "pipeline", &pipelineArray);
        aggregation::Pipeline pipeline(&pipelineArray);
        aggregation::updateHierarchyO(pipeline,
                        tripleCollection_->name(),
                        semweb::IRI_subPropertyOf,
                        assertion.first->iri(),
                        assertion.second->iri());
        bson_append_array_end(&pipelineDoc, &pipelineArray);

        oneCollection_->evalAggregation(&pipelineDoc);
    }

    // update property assertions
    // TODO: below steps are independent, and could run in parallel.
    //       could bake an array of properties into pipeline,
    //       or rather use a bulk operation.
    for(auto &newProperty : visited) {
        bson_reinit(&pipelineDoc);

        bson_t pipelineArray;
        BSON_APPEND_ARRAY_BEGIN(&pipelineDoc, "pipeline", &pipelineArray);
        aggregation::Pipeline pipeline(&pipelineArray);
        aggregation::updateHierarchyP(pipeline,
                        tripleCollection_->name(),
                        semweb::IRI_subPropertyOf,
                        newProperty);
        bson_append_array_end(&pipelineDoc, &pipelineArray);

        oneCollection_->evalAggregation(&pipelineDoc);
    }

    bson_destroy(&pipelineDoc);
}

bool MongoKnowledgeGraph::isTaxonomicProperty(const TermPtr &propertyTerm)
{
    if(propertyTerm->type()==TermType::STRING) {
        return vocabulary_->isTaxonomicProperty(((StringTerm*)propertyTerm.get())->value());
    }
    else {
        return false;
    }
}


// fixture class for testing
class MongoKnowledgeGraphTest : public ::testing::Test {
protected:
    static std::shared_ptr<MongoKnowledgeGraph> kg_;
    static void SetUpTestSuite() {
        kg_ = std::make_shared<MongoKnowledgeGraph>(
                "mongodb://localhost:27017",
                "knowrob",
                "triplesTest");
        kg_->drop();
        kg_->createSearchIndices();
    }
    // void TearDown() override {}
};
std::shared_ptr<MongoKnowledgeGraph> MongoKnowledgeGraphTest::kg_ = {};

TEST_F(MongoKnowledgeGraphTest, LoadLocal)
{
    EXPECT_FALSE(kg_->getCurrentGraphVersion("memory").has_value());
    EXPECT_NO_THROW(kg_->loadTriples("owl/test/memory.owl", knowrob::RDF_XML));
    EXPECT_TRUE(kg_->getCurrentGraphVersion("memory").has_value());
}

TEST_F(MongoKnowledgeGraphTest, QueryTriple)
{
    KB_INFO("foo1");
    semweb::TripleExpression tripleExpression(
        std::make_shared<StringTerm>("http://knowrob.org/kb/mem-test.owl#predicate1"),
        std::make_shared<Variable>("P"),
        std::make_shared<Variable>("O"),
        knowrob::semweb::TripleExpression::EQ);
    KB_INFO("foo2");
    auto cursor = kg_->lookupTriples(tripleExpression);
    const bson_t *resultDoc;

    KB_INFO("foo3");
    auto queryStr = bson_as_json(cursor->query(), nullptr);
    if(queryStr) KB_INFO("query: {}", queryStr);
    else         KB_INFO("query is null!");
    bson_free (queryStr);

    KB_INFO("foo4");
    auto resultAnswer = std::make_shared<Answer>();
    while(cursor->nextAnswer(resultAnswer)) {
        KB_INFO("result: {}", *resultAnswer);
        resultAnswer = std::make_shared<Answer>();
    }
}
