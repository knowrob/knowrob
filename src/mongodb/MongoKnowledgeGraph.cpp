//
// Created by daniel on 01.04.23.
//

#include <gtest/gtest.h>
#include <filesystem>
#include <boost/foreach.hpp>
#include "knowrob/Logger.h"
#include "knowrob/URI.h"
#include "knowrob/mongodb/MongoKnowledgeGraph.h"
#include "knowrob/mongodb/Document.h"
#include "knowrob/mongodb/Cursor.h"
#include "knowrob/mongodb/MongoInterface.h"
#include "knowrob/mongodb/TripleCursor.h"
#include "knowrob/mongodb/aggregation/graph.h"
#include "knowrob/mongodb/aggregation/triples.h"
#include "knowrob/semweb/FramedRDFLiteral.h"
#include "knowrob/semweb/rdf.h"
#include "knowrob/semweb/rdfs.h"
#include "knowrob/semweb/owl.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/queries/QueryEngine.h"
#include "knowrob/backend/KnowledgeGraphManager.h"

#define MONGO_KG_ONE_COLLECTION "one"
#define MONGO_KG_VERSION_KEY "tripledbVersionString"

#define MONGO_KG_SETTING_HOST "host"
#define MONGO_KG_SETTING_PORT "port"
#define MONGO_KG_SETTING_USER "user"
#define MONGO_KG_SETTING_PASSWORD "password"
#define MONGO_KG_SETTING_DB "db"
#define MONGO_KG_SETTING_COLLECTION "collection"
#define MONGO_KG_SETTING_READ_ONLY "read-only"
#define MONGO_KG_SETTING_DROP_GRAPHS "drop_graphs"

#define MONGO_KG_DEFAULT_HOST "localhost"
#define MONGO_KG_DEFAULT_PORT "27017"
#define MONGO_KG_DEFAULT_DB "knowrob"
#define MONGO_KG_DEFAULT_COLLECTION "triples"

using namespace knowrob;
using namespace knowrob::mongo;
using namespace knowrob::semweb;

KNOWROB_BUILTIN_BACKEND("MongoDB", MongoKnowledgeGraph)

// AGGREGATION PIPELINES
bson_t* newPipelineImportHierarchy(const char *collection);

MongoKnowledgeGraph::MongoKnowledgeGraph()
: KnowledgeGraph(),
  isReadOnly_(false)
{
}

MongoKnowledgeGraph::MongoKnowledgeGraph(const char* db_uri, const char* db_name, const char* collectionName)
: KnowledgeGraph(),
  tripleCollection_(MongoInterface::get().connect(db_uri, db_name, collectionName)),
  isReadOnly_(false)
{
    initialize();
    dropGraph("user");
}

bool MongoKnowledgeGraph::loadConfiguration(const boost::property_tree::ptree &config)
{
    tripleCollection_ = connect(config);
    initialize();

    // set isReadOnly_ flag
    auto o_readOnly = config.get_optional<bool>(MONGO_KG_SETTING_READ_ONLY);
    if(o_readOnly.has_value()) {
        isReadOnly_ = o_readOnly.value();
    }

    // auto-drop some named graphs
    auto o_drop_graphs = config.get_child_optional(MONGO_KG_SETTING_DROP_GRAPHS);
    if(o_drop_graphs.has_value()) {
        BOOST_FOREACH(const auto &v, o_drop_graphs.value()) {
            dropGraph(v.second.data());
        }
    } else {
        dropGraph("user");
    }

    return true;
}

const std::string& MongoKnowledgeGraph::dbName() const
{
    return tripleCollection_->dbName();
}

const std::string& MongoKnowledgeGraph::dbURI() const
{
    return tripleCollection_->dbURI();
}

bool MongoKnowledgeGraph::isReadOnly() const
{
    return isReadOnly_;
}

std::shared_ptr<Collection> MongoKnowledgeGraph::connect(const boost::property_tree::ptree &config)
{
    return MongoInterface::get().connect(
              getURI(config).c_str(),
              getDBName(config).c_str(),
              getCollectionName(config).c_str());
}

std::string MongoKnowledgeGraph::getDBName(const boost::property_tree::ptree &config)
{
    static std::string defaultDBName = MONGO_KG_DEFAULT_DB;
    auto o_dbname = config.get_optional<std::string>(MONGO_KG_SETTING_DB);
    return (o_dbname ? o_dbname.value() : defaultDBName);
}

std::string MongoKnowledgeGraph::getCollectionName(const boost::property_tree::ptree &config)
{
    static std::string defaultCollectionName = MONGO_KG_DEFAULT_COLLECTION;
    auto o_collection = config.get_optional<std::string>(MONGO_KG_SETTING_COLLECTION);
    return (o_collection ? o_collection.value() : defaultCollectionName);
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
            tripleCollection_->connection(),
            tripleCollection_->dbName().c_str(),
            MONGO_KG_ONE_COLLECTION);
    // make sure there is one document in the "one" collection.
    if(oneCollection_->empty()) {
        Document oneDoc(bson_new());
        bson_t scopeDoc, timeDoc;
        bson_decimal128_t infinity, zero;
        bson_decimal128_from_string(BSON_DECIMAL128_INF, &infinity);
        bson_decimal128_from_string("0", &zero);
        // TODO: time interval is optional now, right? so we do not need to add the fields here? test it!
        BSON_APPEND_DOCUMENT_BEGIN(oneDoc.bson(), "v_scope", &scopeDoc);
        BSON_APPEND_DOCUMENT_BEGIN(&scopeDoc, "time", &timeDoc);
        BSON_APPEND_DECIMAL128(&timeDoc, "since", &zero);
        BSON_APPEND_DECIMAL128(&timeDoc, "until", &infinity);
        bson_append_document_end(&scopeDoc, &timeDoc);
        bson_append_document_end(oneDoc.bson(), &scopeDoc);
        oneCollection_->storeOne(oneDoc);
    }

    // initialize vocabulary
    StatementData tripleData;
    {
        // iterate over all rdf::type assertions
        TripleCursor cursor(tripleCollection_);
        cursor.filter(Document(BCON_NEW(
            "p", BCON_UTF8(rdf::type.data()))).bson());
        while(cursor.nextTriple(tripleData))
            vocabulary_->addResourceType(tripleData.subject, tripleData.object);
    }
    {
        // iterate over all rdfs::subClassOf assertions
        TripleCursor cursor(tripleCollection_);
        cursor.filter(Document(BCON_NEW(
            "p", BCON_UTF8(rdfs::subClassOf.data()))).bson());
        while(cursor.nextTriple(tripleData))
            vocabulary_->addSubClassOf(tripleData.subject, tripleData.object);
    }
    {
        // iterate over all rdfs::subPropertyOf assertions
        TripleCursor cursor(tripleCollection_);
        cursor.filter(Document(BCON_NEW(
            "p", BCON_UTF8(rdfs::subPropertyOf.data()))).bson());
        while(cursor.nextTriple(tripleData))
            vocabulary_->addSubPropertyOf(tripleData.subject, tripleData.object);
    }
    {
        // iterate over all owl::inverseOf assertions
        TripleCursor cursor(tripleCollection_);
        cursor.filter(Document(BCON_NEW(
            "p", BCON_UTF8(owl::inverseOf.data()))).bson());
        while(cursor.nextTriple(tripleData))
            vocabulary_->setInverseOf(tripleData.subject, tripleData.object);
    }

    // initialize the import hierarchy
    {
        const bson_t *result;
        Cursor cursor(tripleCollection_);
        Document document(newPipelineImportHierarchy(tripleCollection_->name().c_str()));
        cursor.aggregate(document.bson());
        while(cursor.next(&result)) {
            bson_iter_t iter;
            if(!bson_iter_init(&iter, result)) break;
            if(!bson_iter_find(&iter, "importer")) break;
            auto importer = bson_iter_utf8(&iter, nullptr);
            if(!bson_iter_find(&iter, "imported")) break;
            auto imported = bson_iter_utf8(&iter, nullptr);
            importHierarchy_->addDirectImport(importer, imported);
        }
    }
}

void MongoKnowledgeGraph::createSearchIndices()
{
    // TODO: shouldn't fields "graph", "agent", "scope.time.since", "scope.time.until", "confidence",
    //  "uncertain", "occasional" be included in each index?
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
    importHierarchy_->clear();
}

void MongoKnowledgeGraph::dropGraph(const std::string_view &graphName)
{
    KB_INFO("dropping graph with name \"{}\".", graphName);
    tripleCollection_->removeAll(Document(
            BCON_NEW("graph", BCON_UTF8(graphName.data()))));
    // TODO: improve handling of default graph names.
    //       here it is avoided that import relations are forgotten.
    if(graphName != "user" && graphName != "common" && graphName != "test")
        importHierarchy_->removeCurrentGraph(graphName);
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

bson_t* MongoKnowledgeGraph::getSelector(
            const FramedRDFLiteral &tripleExpression,
            bool b_isTaxonomicProperty)
{
    auto doc = bson_new();
    aggregation::appendTripleSelector(doc, tripleExpression, b_isTaxonomicProperty);
    return doc;
}

bool MongoKnowledgeGraph::insert(const StatementData &tripleData)
{
    auto &graph = tripleData.graph ? tripleData.graph : importHierarchy_->defaultGraph();
    TripleLoader loader(graph,
                        tripleCollection_,
                        oneCollection_,
                        vocabulary_);
    loader.loadTriple(tripleData);
    loader.flush();
    updateHierarchy(loader);
    updateTimeInterval(tripleData);
    return true;
}

bool MongoKnowledgeGraph::insert(const std::vector<StatementData> &statements)
{
    auto &graph = importHierarchy_->defaultGraph();
    TripleLoader loader(graph,
                        tripleCollection_,
                        oneCollection_,
                        vocabulary_);
    auto loaderPtr = &loader;
    std::for_each(statements.begin(), statements.end(),
        [loaderPtr](auto &data) {
            loaderPtr->loadTriple(data);
        });
    loader.flush();
    updateHierarchy(loader);

    for(auto &data : statements) updateTimeInterval(data);

    return true;
}

void MongoKnowledgeGraph::removeAll(const FramedRDFLiteral &tripleExpression)
{
    bool b_isTaxonomicProperty = isTaxonomicProperty(tripleExpression.propertyTerm());
    tripleCollection_->removeAll(
        Document(getSelector(tripleExpression, b_isTaxonomicProperty)));
}

void MongoKnowledgeGraph::removeOne(const FramedRDFLiteral &tripleExpression)
{
    bool b_isTaxonomicProperty = isTaxonomicProperty(tripleExpression.propertyTerm());
    tripleCollection_->removeOne(
        Document(getSelector(tripleExpression, b_isTaxonomicProperty)));
}

AnswerCursorPtr MongoKnowledgeGraph::lookup(const FramedRDFLiteral &tripleExpression)
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
}

mongo::AnswerCursorPtr MongoKnowledgeGraph::lookup(const StatementData &tripleData)
{
    return lookup(FramedRDFLiteral(tripleData));
}

mongo::AnswerCursorPtr MongoKnowledgeGraph::lookup(const std::vector<FramedRDFLiteralPtr> &tripleExpressions)
{
    bson_t pipelineDoc = BSON_INITIALIZER;
    bson_t pipelineArray;
    BSON_APPEND_ARRAY_BEGIN(&pipelineDoc, "pipeline", &pipelineArray);
    aggregation::Pipeline pipeline(&pipelineArray);
    aggregation::lookupTriplePaths(pipeline,
                                  tripleCollection_->name(),
                                  vocabulary_,
                                  tripleExpressions);
    bson_append_array_end(&pipelineDoc, &pipelineArray);

    auto cursor = std::make_shared<AnswerCursor>(oneCollection_);
    cursor->aggregate(&pipelineDoc);
    return cursor;
}

void MongoKnowledgeGraph::evaluateQuery(const GraphQueryPtr &query, AnswerBufferPtr &resultStream)
{
    auto channel = AnswerStream::Channel::create(resultStream);
    auto cursor = lookup(query->framedLiterals());

    // limit to one solution if requested
    if(query->flags() & QUERY_FLAG_ONE_SOLUTION) {
        cursor->limit(1);
    }

    while(true) {
        std::shared_ptr<Answer> next = std::make_shared<Answer>();
        if(cursor->nextAnswer(next)) {
            channel->push(next);
        }
        else {
            channel->push(AnswerStream::eos());
            break;
        }
    }
}

AnswerBufferPtr MongoKnowledgeGraph::watchQuery(const GraphQueryPtr &literal)
{
    // TODO implement watchQuery in MongoKnowledgeGraph
    return {};
}

bool MongoKnowledgeGraph::loadFile( //NOLINT
        const std::string_view &uriString,
        TripleFormat format,
        const ModalityFrame &frame)
{
    // TODO: rather format IRI's as e.g. "soma:foo" and store namespaces as part of graph?
    //          or just assume namespace prefixes are unique withing knowrob.
    //          also thinkable to use an integer encoding.
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
    if(!loadURI(loader, *importURI, blankPrefix, format, frame)) {
        KB_WARN("Failed to parse ontology {} ({})", *importURI, uriString);
        return false;
    }
    // update the version record of the ontology
    setCurrentGraphVersion(graphName, resolved, newVersion);
    // update o* and p* fields
    updateHierarchy(loader);
    // load imported ontologies
    for(auto &imported : loader.imports()) loadFile(imported, format, frame);

    return true;
}

void MongoKnowledgeGraph::updateTimeInterval(const StatementData &tripleData)
{
    if(!tripleData.begin.has_value() && !tripleData.end.has_value()) return;
    bool b_isTaxonomicProperty = vocabulary_->isTaxonomicProperty(tripleData.predicate);

    // filter overlapping triples
    TripleCursor cursor(tripleCollection_);
    bson_t selectorDoc = BSON_INITIALIZER;

    StatementData tripleDataCopy(tripleData);
    tripleDataCopy.temporalOperator = TemporalOperator::SOMETIMES;
    FramedRDFLiteral overlappingExpr(tripleDataCopy);
    aggregation::appendTripleSelector(&selectorDoc, overlappingExpr, b_isTaxonomicProperty);
    cursor.filter(&selectorDoc);

    // iterate overlapping triples, remember document ids and compute
    // union of time intervals
    StatementData overlappingTriple;
    std::list<bson_oid_t> documentIDs;
    std::optional<double> begin = tripleData.begin;
    std::optional<double> end = tripleData.end;
    while(cursor.nextTriple(overlappingTriple)) {
        // remember the ID of overlapping documents
        auto &oid = documentIDs.emplace_back();
        bson_oid_init(&oid, nullptr);
        bson_oid_copy((bson_oid_t*)overlappingTriple.documentID, &oid);
        // compute intersection of time interval
        if(overlappingTriple.begin.has_value()) {
            if(begin.has_value()) begin = std::min(begin.value(), overlappingTriple.begin.value());
            else                  begin = overlappingTriple.begin.value();
        }
        if(overlappingTriple.end.has_value()) {
            if(end.has_value()) end = std::max(end.value(), overlappingTriple.end.value());
            else                end = overlappingTriple.end.value();
        }
    }

    if(documentIDs.size()>1) {
        auto &firstOID = documentIDs.front();
        // update time interval of first document ID
        Document updateDoc(bson_new());
        bson_t setDoc, scopeDoc, timeDoc;
        BSON_APPEND_DOCUMENT_BEGIN(updateDoc.bson(), "$set", &setDoc); {
            BSON_APPEND_DOCUMENT_BEGIN(&setDoc, "scope", &scopeDoc);
            BSON_APPEND_DOCUMENT_BEGIN(&scopeDoc, "time", &timeDoc);
            if(begin.has_value()) BSON_APPEND_DOUBLE(&timeDoc, "since", begin.value());
            if(end.has_value())   BSON_APPEND_DOUBLE(&timeDoc, "until", end.value());
            bson_append_document_end(&scopeDoc, &timeDoc);
            bson_append_document_end(&setDoc, &scopeDoc);
        }
        bson_append_document_end(updateDoc.bson(), &setDoc);
        tripleCollection_->update(Document(BCON_NEW("_id", BCON_OID(&firstOID))), updateDoc);
        // remove all other documents
        auto it = documentIDs.begin();
        for(it++; it!=documentIDs.end(); it++) tripleCollection_->removeOne(*it);
    }
}

void MongoKnowledgeGraph::updateHierarchy(TripleLoader &tripleLoader)
{
    // below performs the server-side data transformation for updating hierarchy relations
    // such as rdf::type.
    // However, there are many steps for large ontologies so this might consume some time.
    // TODO: list of parents could be supplied as a constant in aggregation queries below.
    //       currently parents are computed in the query, maybe it would be a bit faster using a constant
    //       baked into the query.

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
                                      rdfs::subClassOf,
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
                                      rdfs::subPropertyOf,
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
                                      rdfs::subPropertyOf,
                                      newProperty);
        bson_append_array_end(&pipelineDoc, &pipelineArray);

        oneCollection_->evalAggregation(&pipelineDoc);
    }

    // update import hierarchy
    for(auto &importString : tripleLoader.imports()) {
        auto resolvedImport = URI::resolve(importString);
        auto importedGraph = getNameFromURI(resolvedImport);
        importHierarchy_->addDirectImport(tripleLoader.graphName(), importedGraph);
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


// AGGREGATION PIPELINES

bson_t* newPipelineImportHierarchy(const char *collection)
{
    return BCON_NEW("pipeline", "[",
        "{", "$match", "{", "p", BCON_UTF8(MONGO_KG_VERSION_KEY), "}", "}",
        "{", "$lookup", "{",
       		"from", BCON_UTF8(collection),
       		"as", BCON_UTF8("x"),
       		"let", "{", "x", BCON_UTF8("$graph"), "}",
       		"pipeline", "[",
                "{", "$match", "{",
       				"p", BCON_UTF8(owl::imports.data()),
       				"$expr", "{", "$eq", "[", BCON_UTF8("$graph"), BCON_UTF8("$$x"), "]", "}",
       			"}", "}",
       			"{", "$project", "{", "o", BCON_INT32(1), "}", "}",
       		"]",
       	"}","}",
        "{", "$unwind", BCON_UTF8("$x"), "}",
        "{", "$lookup", "{",
            "from", BCON_UTF8(collection),
            "as", BCON_UTF8("y"),
            "let", "{", "x", BCON_UTF8("$x.o"), "}",
            "pipeline", "[",
               "{", "$match", "{",
                    "p", BCON_UTF8(MONGO_KG_VERSION_KEY),
                    "$expr", "{", "$eq", "[", BCON_UTF8("$s"), BCON_UTF8("$$x"), "]", "}",
                "}", "}",
                "{", "$project", "{", "graph", BCON_INT32(1), "}", "}",
            "]",
        "}","}",
        "{", "$unwind", BCON_UTF8("$y"), "}",
        "{", "$project", "{", "importer", BCON_UTF8("$graph"), "imported", BCON_UTF8("$y.graph"), "}", "}",
    "]");
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
        kg_->setThreadPool(std::make_shared<ThreadPool>(4));
        kg_->drop();
        kg_->createSearchIndices();
    }
    // void TearDown() override {}
    template <class T>
    std::list<std::shared_ptr<Answer>> lookup(const T &data) {
        auto cursor = kg_->lookup(data);
        std::list<std::shared_ptr<Answer>> out;
        while(true) {
            std::shared_ptr<Answer> next = std::make_shared<Answer>();
            if(cursor->nextAnswer(next)) out.push_back(next);
            else break;
        }
        return out;
    }
    static FramedRDFLiteral parse(const std::string &str) {
        auto p = QueryParser::parsePredicate(str);
        return { p->arguments()[0], p->arguments()[1], p->arguments()[2] };
    }
};
std::shared_ptr<MongoKnowledgeGraph> MongoKnowledgeGraphTest::kg_ = {};

TEST_F(MongoKnowledgeGraphTest, Assert_a_b_c)
{
    StatementData data_abc("a", "b", "c");
    EXPECT_NO_THROW(kg_->insert(data_abc));
    EXPECT_EQ(lookup(data_abc).size(), 1);
    EXPECT_EQ(lookup(parse("triple(x,b,c)")).size(), 0);
    EXPECT_EQ(lookup(parse("triple(a,x,c)")).size(), 0);
    EXPECT_EQ(lookup(parse("triple(a,b,x)")).size(), 0);
    EXPECT_EQ(lookup(parse("triple(A,b,c)")).size(), 1);
    EXPECT_EQ(lookup(parse("triple(A,x,c)")).size(), 0);
    EXPECT_EQ(lookup(parse("triple(a,B,c)")).size(), 1);
    EXPECT_EQ(lookup(parse("triple(x,B,c)")).size(), 0);
    EXPECT_EQ(lookup(parse("triple(a,b,C)")).size(), 1);
    EXPECT_EQ(lookup(parse("triple(x,b,C)")).size(), 0);
}

TEST_F(MongoKnowledgeGraphTest, LoadSOMAandDUL)
{
    EXPECT_FALSE(kg_->getCurrentGraphVersion("swrl").has_value());
    EXPECT_NO_THROW(kg_->loadFile("owl/test/swrl.owl", knowrob::RDF_XML, ModalityFrame()));
    EXPECT_TRUE(kg_->getCurrentGraphVersion("swrl").has_value());

    EXPECT_FALSE(kg_->getCurrentGraphVersion("datatype_test").has_value());
    EXPECT_NO_THROW(kg_->loadFile("owl/test/datatype_test.owl", knowrob::RDF_XML, ModalityFrame()));
    EXPECT_TRUE(kg_->getCurrentGraphVersion("datatype_test").has_value());
}

#define swrl_test_ "http://knowrob.org/kb/swrl_test#"

TEST_F(MongoKnowledgeGraphTest, QueryTriple)
{
    StatementData triple(
        swrl_test_"Adult",
        rdfs::subClassOf.data(),
        swrl_test_"TestThing");
    EXPECT_EQ(lookup(triple).size(), 1);
}

TEST_F(MongoKnowledgeGraphTest, DeleteSubclassOf)
{
    StatementData triple(
        swrl_test_"Adult",
        rdfs::subClassOf.data(),
        swrl_test_"TestThing");
    EXPECT_NO_THROW(kg_->removeAll(FramedRDFLiteral(triple)));
    EXPECT_EQ(lookup(triple).size(), 0);
}

TEST_F(MongoKnowledgeGraphTest, AssertSubclassOf)
{
    StatementData existing(
        swrl_test_"Adult",
        rdfs::subClassOf.data(),
        swrl_test_"TestThing");
    StatementData not_existing(
        swrl_test_"Adult",
        rdfs::subClassOf.data(),
        swrl_test_"Car");
    EXPECT_NO_THROW(kg_->insert(existing));
    EXPECT_EQ(lookup(existing).size(), 1);
    EXPECT_EQ(lookup(not_existing).size(), 0);
}
