/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <gtest/gtest.h>
#include <filesystem>
#include <boost/foreach.hpp>
#include "knowrob/Logger.h"
#include "knowrob/URI.h"
#include "knowrob/db/mongo/MongoKnowledgeGraph.h"
#include "knowrob/db/mongo/Document.h"
#include "knowrob/db/mongo/Cursor.h"
#include "knowrob/db/mongo/MongoInterface.h"
#include "knowrob/db/mongo/TripleCursor.h"
#include "knowrob/db/mongo/aggregation/graph.h"
#include "knowrob/db/mongo/aggregation/triples.h"
#include "knowrob/semweb/FramedTriplePattern.h"
#include "knowrob/semweb/rdf.h"
#include "knowrob/semweb/rdfs.h"
#include "knowrob/semweb/owl.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/db/BackendManager.h"
#include "knowrob/KnowledgeBase.h"
#include "knowrob/queries/AnswerNo.h"
#include "knowrob/db/OntologyParser.h"

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
bson_t *newRelationCounter(const char *collection);

bson_t *newClassCounter(const char *collection);

const std::string MongoKnowledgeGraph::DB_URI_DEFAULT = "mongodb://localhost:27017";
const std::string MongoKnowledgeGraph::DB_NAME_KNOWROB = "knowrob";
const std::string MongoKnowledgeGraph::DB_NAME_TESTS = "knowrob_test";
const std::string MongoKnowledgeGraph::COLL_NAME_TRIPLES = "triples";
const std::string MongoKnowledgeGraph::COLL_NAME_TESTS = "triples_test";

MongoKnowledgeGraph::MongoKnowledgeGraph()
		: DataBackend(),
		  QueryableBackend(),
		  PersistentBackend(),
		  isReadOnly_(false) {
}

bool MongoKnowledgeGraph::init(std::string_view db_uri, std::string_view db_name, std::string_view collectionName) {
	tripleCollection_ = connect(db_uri, db_name, collectionName);
	if (tripleCollection_) {
		initialize();
		dropSessionOrigins();
		return true;
	} else {
		return false;
	}
}

bool MongoKnowledgeGraph::initializeBackend(const ReasonerConfig &config) {
	auto ptree = config.ptree();
	if (!ptree) {
		tripleCollection_ = connect(DB_URI_DEFAULT, DB_NAME_KNOWROB, COLL_NAME_TESTS);
		if (tripleCollection_) {
			initialize();
			dropSessionOrigins();
			return true;
		} else {
			return false;
		}
	}

	tripleCollection_ = connect(*ptree);
	initialize();

	// set isReadOnly_ flag
	auto o_readOnly = ptree->get_optional<bool>(MONGO_KG_SETTING_READ_ONLY);
	if (o_readOnly.has_value()) {
		isReadOnly_ = o_readOnly.value();
	}

	// auto-drop some named graphs
	auto o_drop_graphs = ptree->get_child_optional(MONGO_KG_SETTING_DROP_GRAPHS);
	if (o_drop_graphs.has_value()) {
		BOOST_FOREACH(const auto &v, o_drop_graphs.value()) {
						removeAllWithOrigin(v.second.data());
					}
	} else {
		dropSessionOrigins();
	}

	return true;
}

const std::string &MongoKnowledgeGraph::dbName() const {
	return tripleCollection_->dbName();
}

const std::string &MongoKnowledgeGraph::dbURI() const {
	return tripleCollection_->dbURI();
}

bool MongoKnowledgeGraph::isReadOnly() const {
	return isReadOnly_;
}

std::shared_ptr<Collection> MongoKnowledgeGraph::connect(
		const std::string_view db_uri,
		const std::string_view db_name,
		const std::string_view collectionName) {
	auto coll = MongoInterface::get().connect(db_uri.data(), db_name.data(), collectionName.data());
	if (coll) {
		KB_INFO("[mongodb] connected to {} ({}.{}).", db_uri, db_name, collectionName);
	} else {
		KB_ERROR("[mongodb] failed to connect to {} ({}.{}).", db_uri, db_name, collectionName);
	}
	return coll;
}

std::shared_ptr<Collection> MongoKnowledgeGraph::connect(const boost::property_tree::ptree &config) {
	return connect(getURI(config), getDBName(config), getCollectionName(config));
}

std::string MongoKnowledgeGraph::getDBName(const boost::property_tree::ptree &config) {
	static std::string defaultDBName = MONGO_KG_DEFAULT_DB;
	auto o_dbname = config.get_optional<std::string>(MONGO_KG_SETTING_DB);
	return (o_dbname ? o_dbname.value() : defaultDBName);
}

std::string MongoKnowledgeGraph::getCollectionName(const boost::property_tree::ptree &config) {
	static std::string defaultCollectionName = MONGO_KG_DEFAULT_COLLECTION;
	auto o_collection = config.get_optional<std::string>(MONGO_KG_SETTING_COLLECTION);
	return (o_collection ? o_collection.value() : defaultCollectionName);
}

std::string MongoKnowledgeGraph::getURI(const boost::property_tree::ptree &config) {
	auto o_host = config.get_optional<std::string>(MONGO_KG_SETTING_HOST);
	auto o_port = config.get_optional<std::string>(MONGO_KG_SETTING_PORT);
	auto o_user = config.get_optional<std::string>(MONGO_KG_SETTING_USER);
	auto o_password = config.get_optional<std::string>(MONGO_KG_SETTING_PASSWORD);
	// format URI of the form "mongodb://USER:PW@HOST:PORT"
	std::stringstream uriStream;
	uriStream << "mongodb://";
	if (o_user) {
		uriStream << o_user.value();
		if (o_password) uriStream << ':' << o_password.value();
		uriStream << '@';
	}
	uriStream
			<< (o_host ? o_host.value() : MONGO_KG_DEFAULT_HOST)
			<< ':'
			<< (o_port ? o_port.value() : MONGO_KG_DEFAULT_PORT);
	return uriStream.str();
}

void MongoKnowledgeGraph::initialize() {
	createSearchIndices();
	// a collection with just a single document used for querying
	oneCollection_ = std::make_shared<Collection>(
			tripleCollection_->connection(),
			tripleCollection_->dbName().c_str(),
			MONGO_KG_ONE_COLLECTION);
	// make sure there is one document in the "one" collection.
	if (oneCollection_->empty()) {
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
	FramedTripleView tripleData;
	{
		// iterate over all rdf::type assertions
		TripleCursor cursor(tripleCollection_);
		cursor.filter(Document(BCON_NEW(
									   "p", BCON_UTF8(rdf::type->stringForm().data()))).bson());
		while (cursor.nextTriple(tripleData))
			vocabulary_->addResourceType(tripleData.subject(), tripleData.valueAsString());
	}
	{
		// iterate over all rdfs::subClassOf assertions
		TripleCursor cursor(tripleCollection_);
		cursor.filter(Document(BCON_NEW(
									   "p", BCON_UTF8(rdfs::subClassOf->stringForm().data()))).bson());
		while (cursor.nextTriple(tripleData))
			vocabulary_->addSubClassOf(tripleData.subject(), tripleData.valueAsString());
	}
	{
		// iterate over all rdfs::subPropertyOf assertions
		TripleCursor cursor(tripleCollection_);
		cursor.filter(Document(BCON_NEW(
									   "p", BCON_UTF8(rdfs::subPropertyOf->stringForm().data()))).bson());
		while (cursor.nextTriple(tripleData))
			vocabulary_->addSubPropertyOf(tripleData.subject(), tripleData.valueAsString());
	}
	{
		// iterate over all owl::inverseOf assertions
		TripleCursor cursor(tripleCollection_);
		cursor.filter(Document(BCON_NEW(
									   "p", BCON_UTF8(owl::inverseOf->stringForm().data()))).bson());
		while (cursor.nextTriple(tripleData))
			vocabulary_->setInverseOf(tripleData.subject(), tripleData.valueAsString());
	}

	// query number of assertions of each property.
	// this is useful information for optimizing the query planner.
	{
		const bson_t *result;
		Cursor cursor(tripleCollection_);
		Document document(newRelationCounter(tripleCollection_->name().c_str()));
		cursor.aggregate(document.bson());
		while (cursor.next(&result)) {
			bson_iter_t iter;
			if (!bson_iter_init(&iter, result)) break;
			if (!bson_iter_find(&iter, "property")) break;
			auto property = bson_iter_utf8(&iter, nullptr);
			if (!bson_iter_find(&iter, "count")) break;
			auto count = bson_iter_as_int64(&iter);
			vocabulary_->setFrequency(property, count);
		}
	}
	{
		const bson_t *result;
		Cursor cursor(tripleCollection_);
		Document document(newClassCounter(tripleCollection_->name().c_str()));
		cursor.aggregate(document.bson());
		while (cursor.next(&result)) {
			bson_iter_t iter;
			if (!bson_iter_init(&iter, result)) break;
			if (!bson_iter_find(&iter, "class")) break;
			auto cls = bson_iter_utf8(&iter, nullptr);
			if (!bson_iter_find(&iter, "count")) break;
			auto count = bson_iter_as_int64(&iter);
			vocabulary_->setFrequency(cls, count);
		}
	}

	// initialize the import hierarchy
	for (auto &persistedOrigin: tripleCollection_->distinctValues("graph")) {
		importHierarchy_->addDirectImport(importHierarchy_->ORIGIN_SYSTEM, persistedOrigin);
	}
}

void MongoKnowledgeGraph::createSearchIndices() {
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

void MongoKnowledgeGraph::drop() {
	tripleCollection_->drop();
	vocabulary_ = std::make_shared<semweb::Vocabulary>();
	importHierarchy_->clear();
}

bool MongoKnowledgeGraph::dropOrigin(std::string_view graphName) {
	KB_INFO("[mongodb] dropping triples with origin \"{}\".", graphName);
	tripleCollection_->removeAll(Document(
			BCON_NEW("graph", BCON_UTF8(graphName.data()))));
	return true;
}

bool MongoKnowledgeGraph::dropSessionOrigins() {
	// TODO: rather iterate over all children of ORIGIN_SESSION, and drop all of them
	// TODO: I think it would be better if this is done centrally
	return dropOrigin(semweb::ImportHierarchy::ORIGIN_USER) &&
		   dropOrigin(semweb::ImportHierarchy::ORIGIN_REASONER) &&
		   dropOrigin(semweb::ImportHierarchy::ORIGIN_SESSION);
}

void MongoKnowledgeGraph::setVersionOfOrigin(std::string_view origin, std::string_view version) {
	KB_INFO("[mongodb] set version for origin \"{}\" to \"{}\".", origin, version);
	tripleCollection_->storeOne(Document(BCON_NEW(
												 "s", BCON_UTF8(origin.data()),
												 "p", BCON_UTF8(MONGO_KG_VERSION_KEY),
												 "o", BCON_UTF8(version.data()),
												 "graph", BCON_UTF8(origin.data()))));
}

std::optional<std::string> MongoKnowledgeGraph::getVersionOfOrigin(std::string_view origin) {
	auto document = Document(BCON_NEW(
									 "p", BCON_UTF8(MONGO_KG_VERSION_KEY),
									 "graph", BCON_UTF8(origin.data())));
	const bson_t *result;
	Cursor cursor(tripleCollection_);
	cursor.limit(1);
	cursor.filter(document.bson());
	if (cursor.next(&result)) {
		bson_iter_t iter;
		if (bson_iter_init(&iter, result) && bson_iter_find(&iter, "o")) {
			return std::string(bson_iter_utf8(&iter, nullptr));
		}
	}
	// no version is loaded yet
	return {};
}

bson_t *MongoKnowledgeGraph::getSelector(
		const FramedTriplePattern &tripleExpression,
		bool b_isTaxonomicProperty) {
	auto doc = bson_new();
	aggregation::appendTripleSelector(doc, tripleExpression, b_isTaxonomicProperty, importHierarchy_);
	return doc;
}

bson_t *MongoKnowledgeGraph::getSelector(
		const FramedTriple &triple,
		bool b_isTaxonomicProperty) {
	return getSelector(FramedTriplePattern(triple), b_isTaxonomicProperty);
}

bool MongoKnowledgeGraph::insertOne(const FramedTriple &tripleData) {
	auto &fallbackOrigin = importHierarchy_->defaultGraph();
	bool isTaxonomic = isTaxonomicProperty(tripleData.predicate());
	auto document = createTripleDocument(tripleData, fallbackOrigin, isTaxonomic);
	tripleCollection_->storeOne(Document(document));

	if (semweb::isSubClassOfIRI(tripleData.predicate())) {
		updateHierarchy({{tripleData.subject(), tripleData.valueAsString()}}, {});
	} else if (semweb::isSubPropertyOfIRI(tripleData.predicate())) {
		updateHierarchy({}, {{tripleData.subject(), tripleData.valueAsString()}});
	}
	updateTimeInterval(tripleData);
	return true;
}

bool MongoKnowledgeGraph::insertAll(const semweb::TripleContainerPtr &triples) {
	// only used in case triples do not specify origin field
	auto &fallbackOrigin = importHierarchy_->defaultGraph();
	auto bulk = tripleCollection_->createBulkOperation();
	struct TaxonomyAssertions {
		std::vector<StringPair> subClassAssertions;
		std::vector<StringPair> subPropertyAssertions;
	} tAssertions;

	std::for_each(triples->begin(), triples->end(),
				  [&](auto &data) {
					  bool isTaxonomic = isTaxonomicProperty(data->predicate().data());

					  auto document = createTripleDocument(*data, fallbackOrigin, isTaxonomic);
					  bulk->pushInsert(document);
					  bson_free(document);

					  if (semweb::isSubClassOfIRI(data->predicate())) {
						  tAssertions.subClassAssertions.emplace_back(data->subject(), data->valueAsString());
					  } else if (semweb::isSubPropertyOfIRI(data->predicate())) {
						  tAssertions.subPropertyAssertions.emplace_back(data->subject(), data->valueAsString());
					  }
				  });
	bulk->execute();

	updateHierarchy(tAssertions.subClassAssertions, tAssertions.subPropertyAssertions);
	for (auto &data: *triples) updateTimeInterval(*data);

	return true;
}

bool MongoKnowledgeGraph::removeOne(const FramedTriple &triple) {
	bool b_isTaxonomicProperty = isTaxonomicProperty(triple.predicate());
	tripleCollection_->removeOne(Document(getSelector(triple, b_isTaxonomicProperty)));
	return true;
}

bool MongoKnowledgeGraph::removeAll(const semweb::TripleContainerPtr &triples) {
	auto bulk = tripleCollection_->createBulkOperation();
	std::for_each(triples->begin(), triples->end(),
				  [this, bulk](auto &data) {
					  bool isTaxonomic = isTaxonomicProperty(data->predicate());
					  auto document = getSelector(*data, isTaxonomic);
					  bulk->pushRemoveOne(document);
					  bson_free(document);
				  });
	bulk->execute();

	// FIXME: handle hierarchy updates

	return true;
}

bool MongoKnowledgeGraph::removeAllWithOrigin(std::string_view graphName) {
	return dropOrigin(graphName);
}

bool MongoKnowledgeGraph::removeAllMatching(const FramedTriplePatternPtr &query) {
	static const bool doMatchMany = true;
	bool b_isTaxonomicProperty = isTaxonomicProperty(query->propertyTerm());
	if (doMatchMany) {
		tripleCollection_->removeAll(Document(getSelector(*query, b_isTaxonomicProperty)));
	} else {
		tripleCollection_->removeOne(Document(getSelector(*query, b_isTaxonomicProperty)));
	}
	// TODO: does mongo return number of documents removed?
	return true;
}

AnswerCursorPtr MongoKnowledgeGraph::lookup(const FramedTriplePattern &tripleExpression) {
	bson_t pipelineDoc = BSON_INITIALIZER;
	bson_t pipelineArray;

	BSON_APPEND_ARRAY_BEGIN(&pipelineDoc, "pipeline", &pipelineArray);
	aggregation::Pipeline pipeline(&pipelineArray);
	{
		// append lookup stages to pipeline
		aggregation::TripleLookupData lookupData(&tripleExpression);
		// indicate that no variables in tripleExpression may have been instantiated
		// by a previous step to allow for some optimizations.
		lookupData.mayHasMoreGroundings = false;
		aggregation::lookupTriple(pipeline, tripleCollection_->name(), vocabulary_, importHierarchy_, lookupData);
	}
	bson_append_array_end(&pipelineDoc, &pipelineArray);

	auto cursor = std::make_shared<AnswerCursor>(oneCollection_);
	cursor->aggregate(&pipelineDoc);
	return cursor;
}

mongo::AnswerCursorPtr MongoKnowledgeGraph::lookup(const FramedTriple &tripleData) {
	return lookup(FramedTriplePattern(tripleData));
}

mongo::AnswerCursorPtr
MongoKnowledgeGraph::lookup(const std::vector<FramedTriplePatternPtr> &tripleExpressions, uint32_t limit) {
	bson_t pipelineDoc = BSON_INITIALIZER;
	bson_t pipelineArray;
	BSON_APPEND_ARRAY_BEGIN(&pipelineDoc, "pipeline", &pipelineArray);
	aggregation::Pipeline pipeline(&pipelineArray);
	aggregation::lookupTriplePaths(pipeline,
								   tripleCollection_->name(),
								   vocabulary_,
								   importHierarchy_,
								   tripleExpressions);
	if (limit > 0) {
		pipeline.limit(limit);
	}
	bson_append_array_end(&pipelineDoc, &pipelineArray);

	auto cursor = std::make_shared<AnswerCursor>(oneCollection_);
	cursor->aggregate(&pipelineDoc);
	return cursor;
}

void MongoKnowledgeGraph::evaluateQuery(const ConjunctiveQueryPtr &query, const TokenBufferPtr &resultStream) {
	static const auto edbTerm = Atom::Tabled("EDB");

	auto channel = TokenStream::Channel::create(resultStream);

	try {
		uint32_t limit = (query->flags() & QUERY_FLAG_ONE_SOLUTION) ? 1 : 0;
		auto cursor = lookup(query->literals(), limit);

		// NOTE: for some reason below causes a cursor error. looks like a bug in libmongoc to me!
		//       anyways, we add instead a $limit stage in the aggregation pipeline.
		//if(query->flags() & QUERY_FLAG_ONE_SOLUTION) { cursor->limit(1); }

		bool hasPositiveAnswer = false;
		while (true) {
			auto next = std::make_shared<AnswerYes>();
			next->setReasonerTerm(edbTerm);

			if (cursor->nextAnswer(next, query->literals())) {
				channel->push(next);
				hasPositiveAnswer = true;
			} else {
				if (!hasPositiveAnswer) {
					// send one negative answer if no positive answer was found
					auto negativeAnswer = std::make_shared<AnswerNo>();
					negativeAnswer->setReasonerTerm(edbTerm);
					// the answer is uncertain as we only were not able to obtain a positive answer
					// which does not mean that there is no positive answer.
					negativeAnswer->setIsUncertain(true);
					// add ungrounded literals to negative answer.
					// but at the moment the information is lost at which literal the query failed.
					// TODO: would be great if we could report the failing literal.
					//       but seems hard to provide this information in the current framework.
					//       it could be queried here, but that seems a bit costly.
					// well at least we know if it is a single literal.
					if (query->literals().size() == 1) {
						negativeAnswer->addUngrounded(query->literals().front()->predicate(),
													  query->literals().front()->isNegated());
					}
					channel->push(negativeAnswer);
				}
				channel->push(EndOfEvaluation::get());
				break;
			}
		}
	}
	catch (const std::exception &e) {
		// make sure EOS is pushed to the stream
		channel->push(EndOfEvaluation::get());
		throw;
	}
}

TokenBufferPtr MongoKnowledgeGraph::watchQuery(const ConjunctiveQueryPtr &literal) {
	// TODO implement watchQuery in MongoKnowledgeGraph
	return {};
}

void MongoKnowledgeGraph::updateTimeInterval(const FramedTriple &tripleData) {
	if (!tripleData.begin().has_value() && !tripleData.end().has_value()) return;
	bool b_isTaxonomicProperty = vocabulary_->isTaxonomicProperty(tripleData.predicate());

	// filter overlapping triples
	TripleCursor cursor(tripleCollection_);
	bson_t selectorDoc = BSON_INITIALIZER;

	FramedTriplePattern overlappingExpr(tripleData);
	overlappingExpr.setTemporalOperator(TemporalOperator::SOMETIMES);
	aggregation::appendTripleSelector(&selectorDoc, overlappingExpr, b_isTaxonomicProperty, importHierarchy_);
	cursor.filter(&selectorDoc);

	// iterate overlapping triples, remember document ids and compute
	// union of time intervals
	FramedTripleView overlappingTriple;
	std::list<bson_oid_t> documentIDs;
	std::optional<double> begin = tripleData.begin();
	std::optional<double> end = tripleData.end();
	const bson_oid_t *overlappingOID = nullptr;
	while (cursor.nextTriple(overlappingTriple, &overlappingOID)) {
		// remember the ID of overlapping documents
		auto &oid = documentIDs.emplace_back();
		bson_oid_init(&oid, nullptr);
		bson_oid_copy(overlappingOID, &oid);
		// compute intersection of time interval
		if (overlappingTriple.begin().has_value()) {
			if (begin.has_value()) begin = std::min(begin.value(), overlappingTriple.begin().value());
			else begin = overlappingTriple.begin();
		}
		if (overlappingTriple.end().has_value()) {
			if (end.has_value()) end = std::max(end.value(), overlappingTriple.end().value());
			else end = overlappingTriple.end();
		}
	}

	if (documentIDs.size() > 1) {
		auto &firstOID = documentIDs.front();
		// update time interval of first document ID
		Document updateDoc(bson_new());
		bson_t setDoc, scopeDoc, timeDoc;
		BSON_APPEND_DOCUMENT_BEGIN(updateDoc.bson(), "$set", &setDoc);
		{
			BSON_APPEND_DOCUMENT_BEGIN(&setDoc, "scope", &scopeDoc);
			BSON_APPEND_DOCUMENT_BEGIN(&scopeDoc, "time", &timeDoc);
			if (begin.has_value()) BSON_APPEND_DOUBLE(&timeDoc, "since", begin.value());
			if (end.has_value()) BSON_APPEND_DOUBLE(&timeDoc, "until", end.value());
			bson_append_document_end(&scopeDoc, &timeDoc);
			bson_append_document_end(&setDoc, &scopeDoc);
		}
		bson_append_document_end(updateDoc.bson(), &setDoc);
		tripleCollection_->update(Document(BCON_NEW("_id", BCON_OID(&firstOID))), updateDoc);
		// remove all other documents
		auto it = documentIDs.begin();
		for (it++; it != documentIDs.end(); it++) tripleCollection_->removeOne(*it);
	}
}

void MongoKnowledgeGraph::updateHierarchy(
		const std::vector<StringPair> &subClassAssertions,
		const std::vector<StringPair> &subPropertyAssertions) {
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
	for (auto &assertion: subClassAssertions) {
		bson_reinit(&pipelineDoc);

		bson_t pipelineArray;
		BSON_APPEND_ARRAY_BEGIN(&pipelineDoc, "pipeline", &pipelineArray);
		aggregation::Pipeline pipeline(&pipelineArray);
		aggregation::updateHierarchyO(pipeline,
									  tripleCollection_->name(),
									  rdfs::subClassOf->stringForm(),
									  assertion.first,
									  assertion.second);
		bson_append_array_end(&pipelineDoc, &pipelineArray);

		oneCollection_->evalAggregation(&pipelineDoc);
	}

	// update property hierarchy.
	// unfortunately must be done step-by-step as it is undefined yet in mongo
	// if it's possible to access $merge results in following pipeline iterations
	// via e.g. $lookup.
	std::set<std::string_view> visited;
	for (auto &assertion: subPropertyAssertions) {
		visited.insert(assertion.first);
		bson_reinit(&pipelineDoc);

		bson_t pipelineArray;
		BSON_APPEND_ARRAY_BEGIN(&pipelineDoc, "pipeline", &pipelineArray);
		aggregation::Pipeline pipeline(&pipelineArray);
		aggregation::updateHierarchyO(pipeline,
									  tripleCollection_->name(),
									  rdfs::subPropertyOf->stringForm(),
									  assertion.first,
									  assertion.second);
		bson_append_array_end(&pipelineDoc, &pipelineArray);

		oneCollection_->evalAggregation(&pipelineDoc);
	}

	// update property assertions
	// TODO: below steps are independent, and could run in parallel.
	//       could bake an array of properties into pipeline,
	//       or rather use a bulk operation.
	for (auto &newProperty: visited) {
		bson_reinit(&pipelineDoc);

		bson_t pipelineArray;
		BSON_APPEND_ARRAY_BEGIN(&pipelineDoc, "pipeline", &pipelineArray);
		aggregation::Pipeline pipeline(&pipelineArray);
		aggregation::updateHierarchyP(pipeline,
									  tripleCollection_->name(),
									  rdfs::subPropertyOf->stringForm(),
									  newProperty);
		bson_append_array_end(&pipelineDoc, &pipelineArray);

		oneCollection_->evalAggregation(&pipelineDoc);
	}

	bson_destroy(&pipelineDoc);
}

bool MongoKnowledgeGraph::isTaxonomicProperty(std::string_view property) {
	return vocabulary_->isTaxonomicProperty(property);
}

bool MongoKnowledgeGraph::isTaxonomicProperty(const TermPtr &propertyTerm) {
	if (propertyTerm->termType() == TermType::ATOMIC) {
		return vocabulary_->isTaxonomicProperty(((Atomic *) propertyTerm.get())->stringForm());
	} else {
		return false;
	}
}

static inline void appendXSDLiteral(bson_t *tripleDoc, const FramedTriple &tripleData) {
	auto xsdType = tripleData.xsdType().has_value() ? tripleData.xsdType().value() : XSDType::STRING;
	switch (xsdType) {
		case XSDType::STRING:
			BSON_APPEND_UTF8(tripleDoc, "o", tripleData.valueAsString().data());
			break;
		case XSDType::DOUBLE:
			BSON_APPEND_DOUBLE(tripleDoc, "o", tripleData.valueAsDouble());
			break;
		case XSDType::FLOAT:
			BSON_APPEND_DOUBLE(tripleDoc, "o", tripleData.valueAsFloat());
			break;
		case XSDType::LONG:
			BSON_APPEND_INT64(tripleDoc, "o", tripleData.valueAsLong());
			break;
		case XSDType::NON_NEGATIVE_INTEGER:
		case XSDType::INTEGER:
			BSON_APPEND_INT32(tripleDoc, "o", tripleData.valueAsInt());
			break;
		case XSDType::SHORT:
			BSON_APPEND_INT32(tripleDoc, "o", tripleData.valueAsShort());
			break;
		case XSDType::BOOLEAN:
			BSON_APPEND_BOOL(tripleDoc, "o", tripleData.valueAsBoolean());
			break;
		case XSDType::UNSIGNED_INT:
			BSON_APPEND_INT32(tripleDoc, "o", tripleData.valueAsUnsignedInt());
			break;
		case XSDType::UNSIGNED_SHORT:
			BSON_APPEND_INT32(tripleDoc, "o", tripleData.valueAsUnsignedShort());
			break;
		case XSDType::UNSIGNED_LONG:
			BSON_APPEND_INT64(tripleDoc, "o", tripleData.valueAsUnsignedLong());
			break;
		case XSDType::LAST:
			break;
	}
}

bson_t *MongoKnowledgeGraph::createTripleDocument(const FramedTriple &tripleData,
												  const std::string &fallbackOrigin,
												  bool isTaxonomic) {
	bson_t parentsArray;
	uint32_t arrIndex = 0;
	auto counterPtr = &arrIndex;

	bson_t *tripleDoc = bson_new();
	BSON_APPEND_UTF8(tripleDoc, "s", tripleData.subject().data());
	BSON_APPEND_UTF8(tripleDoc, "p", tripleData.predicate().data());

	if (isTaxonomic) {
		if (tripleData.isObjectIRI() || tripleData.isObjectBlank()) {
			auto objectIRI = tripleData.valueAsString();
			BSON_APPEND_UTF8(tripleDoc, "o", objectIRI.data());
			// also create a field "o*" with the parents of the object
			BSON_APPEND_ARRAY_BEGIN(tripleDoc, "o*", &parentsArray);
			auto parentsPtr = &parentsArray;
			if (vocabulary_->isDefinedProperty(objectIRI)) {
				vocabulary_->getDefinedProperty(objectIRI)->forallParents(
						[parentsPtr, counterPtr](const auto &parent) {
							auto counterKey = std::to_string((*counterPtr)++);
							BSON_APPEND_UTF8(parentsPtr, counterKey.c_str(), parent.iri().data());
						});
			} else if (vocabulary_->isDefinedClass(objectIRI)) {
				// read parents array
				vocabulary_->getDefinedClass(objectIRI)->forallParents(
						[parentsPtr, counterPtr](const auto &parent) {
							auto counterKey = std::to_string((*counterPtr)++);
							BSON_APPEND_UTF8(parentsPtr, counterKey.c_str(), parent.iri().data());
						});
			} else {
				BSON_APPEND_UTF8(&parentsArray, "0", objectIRI.data());
			}
			bson_append_array_end(tripleDoc, &parentsArray);
		} else {
			appendXSDLiteral(tripleDoc, tripleData);
		}
	} else {
		if (tripleData.isObjectIRI() || tripleData.isObjectBlank()) {
			BSON_APPEND_UTF8(tripleDoc, "o", tripleData.valueAsString().data());
		} else {
			appendXSDLiteral(tripleDoc, tripleData);
		}
		// read parents array
		BSON_APPEND_ARRAY_BEGIN(tripleDoc, "p*", &parentsArray);
		auto parentsPtr = &parentsArray;
		vocabulary_->defineProperty(tripleData.predicate())->forallParents(
				[parentsPtr, counterPtr](const auto &parent) {
					auto counterKey = std::to_string((*counterPtr)++);
					BSON_APPEND_UTF8(parentsPtr, counterKey.c_str(), parent.iri().data());
				});
		bson_append_array_end(tripleDoc, &parentsArray);
	}

	if (tripleData.graph()) {
		BSON_APPEND_UTF8(tripleDoc, "graph", tripleData.graph().value().data());
	} else {
		BSON_APPEND_UTF8(tripleDoc, "graph", fallbackOrigin.c_str());
	}

	if (tripleData.agent())
		BSON_APPEND_UTF8(tripleDoc, "agent", tripleData.agent().value().data());

	bool isBelief = false;
	if (tripleData.confidence().has_value()) {
		BSON_APPEND_DOUBLE(tripleDoc, "confidence", tripleData.confidence().value());
		isBelief = true;
	} else if (tripleData.epistemicOperator().has_value()) {
		isBelief = (tripleData.epistemicOperator().value() == EpistemicOperator::BELIEF);
	}
	if (isBelief) {
		// flag the statement as "uncertain"
		BSON_APPEND_BOOL(tripleDoc, "uncertain", true);
	}

	if (tripleData.temporalOperator().has_value() &&
		tripleData.temporalOperator().value() == TemporalOperator::SOMETIMES) {
		// flag the statement as "occasional", meaning it is only known that it was true at some past instants
		BSON_APPEND_BOOL(tripleDoc, "occasional", true);
	}

	if (tripleData.begin().has_value() || tripleData.end().has_value()) {
		bson_t scopeDoc, timeDoc;
		BSON_APPEND_DOCUMENT_BEGIN(tripleDoc, "scope", &scopeDoc);
		BSON_APPEND_DOCUMENT_BEGIN(&scopeDoc, "time", &timeDoc);
		if (tripleData.begin().has_value()) BSON_APPEND_DOUBLE(&timeDoc, "since", tripleData.begin().value());
		if (tripleData.end().has_value()) BSON_APPEND_DOUBLE(&timeDoc, "until", tripleData.end().value());
		bson_append_document_end(&scopeDoc, &timeDoc);
		bson_append_document_end(tripleDoc, &scopeDoc);
	}

	return tripleDoc;
}

// AGGREGATION PIPELINES

bson_t *newRelationCounter(const char *collection) {
	return BCON_NEW("pipeline", "[",
					"{", "$match", "{",
					"p", BCON_UTF8(rdf::type->stringForm().data()),
					"$expr", "{", "$in", "[", "$o", "[",
					BCON_UTF8(owl::ObjectProperty->stringForm().data()),
					BCON_UTF8(owl::DatatypeProperty->stringForm().data()),
					"]", "]", "}",
					"}", "}",
					"{", "$group", "{",
					"_id", BCON_NULL,
					"s", "{", "$addToSet", "$s", "}",
					"}", "}",
					"{", "$unwind", "$s", "}",
					"{", "$lookup", "{",
					"from", BCON_UTF8(collection),
					"as", BCON_UTF8("x"),
					"let", "{", "outer", BCON_UTF8("$s"), "}",
					"pipeline", "[",
					"{", "$match", "{",
					"$expr", "{", "$eq", "[", BCON_UTF8("$p"), BCON_UTF8("$$outer"), "]", "}",
					"}", "}",
					"]",
					"}", "}",
					"{", "$project", "{",
					"property", BCON_UTF8("$s"),
					"count", "{", "$size", BCON_UTF8("$x"), "}",
					"}", "}",
					"{", "$match", "{",
					"$expr", "{", "$gt", "[", "$count", BCON_INT32(0), "]", "}",
					"}", "}",
					"]"
	);
}

bson_t *newClassCounter(const char *collection) {
	// TODO: skip classes that start with "_" character as these are blank nodes
	return BCON_NEW("pipeline", "[",
					"{", "$match", "{",
					"p", BCON_UTF8(rdf::type->stringForm().data()),
					"$expr", "{", "$in", "[", "$o", "[",
					BCON_UTF8(owl::Class->stringForm().data()),
					"]", "]", "}",
					"}", "}",
					"{", "$group", "{",
					"_id", BCON_NULL,
					"s", "{", "$addToSet", "$s", "}",
					"}", "}",
					"{", "$unwind", "$s", "}",
					"{", "$lookup", "{",
					"from", BCON_UTF8(collection),
					"as", BCON_UTF8("x"),
					"let", "{", "outer", BCON_UTF8("$s"), "}",
					"pipeline", "[",
					"{", "$match", "{",
					"$expr", "{", "$eq", "[", BCON_UTF8("$o"), BCON_UTF8("$$outer"), "]", "}",
					"}", "}",
					"]",
					"}", "}",
					"{", "$project", "{",
					"class", BCON_UTF8("$s"),
					"count", "{", "$size", BCON_UTF8("$x"), "}",
					"}", "}",
					"{", "$match", "{",
					"$expr", "{", "$gt", "[", "$count", BCON_INT32(0), "]", "}",
					"}", "}",
					"]"
	);
}


// fixture class for testing
class MongoKnowledgeGraphTest : public ::testing::Test {
protected:
	static std::shared_ptr<MongoKnowledgeGraph> kg_;
	static std::shared_ptr<semweb::Vocabulary> vocabulary_;

	static void SetUpTestSuite() {
		vocabulary_ = std::make_shared<semweb::Vocabulary>();

		kg_ = std::make_shared<MongoKnowledgeGraph>();
		kg_->setVocabulary(vocabulary_);
		kg_->setImportHierarchy(std::make_shared<semweb::ImportHierarchy>());
		kg_->init(
				MongoKnowledgeGraph::DB_URI_DEFAULT,
				MongoKnowledgeGraph::DB_NAME_KNOWROB,
				MongoKnowledgeGraph::COLL_NAME_TESTS);
		kg_->drop();
		kg_->createSearchIndices();
	}

	// void TearDown() override {}
	template<class T>
	std::list<std::shared_ptr<AnswerYes>> lookup(const T &data) {
		auto cursor = kg_->lookup(data);
		std::list<std::shared_ptr<AnswerYes>> out;
		while (true) {
			auto next = std::make_shared<AnswerYes>();
			if (cursor->nextAnswer(next, {})) {
				out.push_back(next);
			} else {
				break;
			}
		}
		return out;
	}

	static FramedTriplePattern parse(const std::string &str) {
		auto p = QueryParser::parsePredicate(str);
		return {p->arguments()[0], p->arguments()[1], p->arguments()[2], false};
	}

	bool loadOntology(std::string_view path) {
		auto resolved = URI::resolve(path);
		auto origin = DataSource::getNameFromURI(resolved);
		auto vocab = vocabulary_;
		OntologyParser parser(resolved, semweb::TripleFormat::RDF_XML, 100);
		// filter is called for each triple, if it returns false, the triple is skipped
		parser.setFilter([vocab](const FramedTriple &triple) {
			return !vocab->isAnnotationProperty(triple.predicate());
		});
		// define a prefix for naming blank nodes
		parser.setBlankPrefix(std::string("_") + origin);
		auto result = parser.run([this](const semweb::TripleContainerPtr &tripleContainer) {
			kg_->insertAll(tripleContainer);
		});
		if (result) {
			return true;
		} else {
			KB_WARN("Failed to parse ontology {} ({})", resolved, origin);
			return false;
		}
	}
};

std::shared_ptr<MongoKnowledgeGraph> MongoKnowledgeGraphTest::kg_ = {};
std::shared_ptr<semweb::Vocabulary> MongoKnowledgeGraphTest::vocabulary_ = {};

TEST_F(MongoKnowledgeGraphTest, Assert_a_b_c) {
	FramedTripleCopy data_abc("a", "b", "c");
	EXPECT_NO_THROW(kg_->insertOne(data_abc));
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

TEST_F(MongoKnowledgeGraphTest, LoadSOMAandDUL) {
	EXPECT_NO_THROW(loadOntology("owl/test/swrl.owl"));
	EXPECT_NO_THROW(loadOntology("owl/test/datatype_test.owl"));
}

#define swrl_test_ "http://knowrob.org/kb/swrl_test#"

TEST_F(MongoKnowledgeGraphTest, QueryTriple) {
	FramedTripleCopy triple(
			swrl_test_"Adult",
			rdfs::subClassOf->stringForm().data(),
			swrl_test_"TestThing");
	EXPECT_EQ(lookup(triple).size(), 1);
}

TEST_F(MongoKnowledgeGraphTest, QueryNegatedTriple) {
	auto negated = std::make_shared<FramedTriplePattern>(
			QueryParser::parsePredicate("p(x,y)"), true);
	EXPECT_EQ(lookup(*negated).size(), 1);
	FramedTripleCopy statement("x", "p", "y");
	EXPECT_NO_THROW(kg_->insertOne(statement));
	EXPECT_EQ(lookup(*negated).size(), 0);
}

TEST_F(MongoKnowledgeGraphTest, DeleteSubclassOf) {
	FramedTripleCopy triple(
			swrl_test_"Adult",
			rdfs::subClassOf->stringForm().data(),
			swrl_test_"TestThing");
	EXPECT_NO_THROW(kg_->removeOne(triple));
	EXPECT_EQ(lookup(triple).size(), 0);
}

TEST_F(MongoKnowledgeGraphTest, AssertSubclassOf) {
	FramedTripleCopy existing(
			swrl_test_"Adult",
			rdfs::subClassOf->stringForm().data(),
			swrl_test_"TestThing");
	FramedTripleCopy not_existing(
			swrl_test_"Adult",
			rdfs::subClassOf->stringForm().data(),
			swrl_test_"Car");
	EXPECT_NO_THROW(kg_->insertOne(existing));
	EXPECT_EQ(lookup(existing).size(), 1);
	EXPECT_EQ(lookup(not_existing).size(), 0);
}

TEST_F(MongoKnowledgeGraphTest, Knowledge) {
	FramedTripleCopy statement(swrl_test_"Lea", swrl_test_"hasName", "X");
	statement.setEpistemicOperator(EpistemicOperator::KNOWLEDGE);
	EXPECT_EQ(lookup(statement).size(), 0);
	EXPECT_NO_THROW(kg_->insertOne(statement));
	EXPECT_EQ(lookup(statement).size(), 1);
	statement.setEpistemicOperator(EpistemicOperator::BELIEF);
	EXPECT_EQ(lookup(statement).size(), 1);
}

TEST_F(MongoKnowledgeGraphTest, KnowledgeOfAgent) {
	// assert knowledge of a named agent
	FramedTripleCopy statement(swrl_test_"Lea", swrl_test_"hasName", "Y");
	statement.setEpistemicOperator(EpistemicOperator::KNOWLEDGE);
	statement.setAgent("agent_a");
	EXPECT_EQ(lookup(statement).size(), 0);
	EXPECT_NO_THROW(kg_->insertOne(statement));
	EXPECT_EQ(lookup(statement).size(), 1);
	for (const auto &solution: lookup(statement)) {
		EXPECT_TRUE(solution->isCertain());
	}
	// the statement is not known to be true for other agents
	statement.setAgent("agent_b");
	EXPECT_EQ(lookup(statement).size(), 0);
}

TEST_F(MongoKnowledgeGraphTest, Belief) {
	// assert uncertain statement
	FramedTripleCopy statement(swrl_test_"Lea", swrl_test_"hasName", "Lea");
	statement.setEpistemicOperator(EpistemicOperator::BELIEF);
	EXPECT_EQ(lookup(statement).size(), 0);
	EXPECT_NO_THROW(kg_->insertOne(statement));
	EXPECT_EQ(lookup(statement).size(), 1);
	for (const auto &solution: lookup(statement)) {
		EXPECT_TRUE(solution->isUncertain());
	}
	// statement is filtered if knowledge operator is selected
	statement.setEpistemicOperator(EpistemicOperator::KNOWLEDGE);
	EXPECT_EQ(lookup(statement).size(), 0);
}

TEST_F(MongoKnowledgeGraphTest, WithConfidence) {
	// assert uncertain statement with confidence=0.5
	FramedTripleCopy statement(swrl_test_"Lea", swrl_test_"hasName", "A");
	statement.setEpistemicOperator(EpistemicOperator::BELIEF);
	statement.setConfidence(0.5);
	EXPECT_EQ(lookup(statement).size(), 0);
	EXPECT_NO_THROW(kg_->insertOne(statement));
	EXPECT_EQ(lookup(statement).size(), 1);
	for (const auto &solution: lookup(statement)) {
		EXPECT_TRUE(solution->isUncertain());
	}
	// confidence threshold of 0.0 does not filter the statement
	statement.setConfidence(0.0);
	EXPECT_EQ(lookup(statement).size(), 1);
	// confidence threshold of 0.9 filters the statement
	statement.setConfidence(0.9);
	EXPECT_EQ(lookup(statement).size(), 0);
}

TEST_F(MongoKnowledgeGraphTest, WithTimeInterval) {
	// assert a statement with time interval [5,10]
	FramedTripleCopy statement(swrl_test_"Rex", swrl_test_"hasName", "Rex");
	statement.setBegin(5.0);
	statement.setEnd(10.0);
	EXPECT_EQ(lookup(statement).size(), 0);
	EXPECT_NO_THROW(kg_->insertOne(statement));
	EXPECT_EQ(lookup(statement).size(), 1);
	for (const auto &solution: lookup(statement)) {
		EXPECT_TRUE(solution->isCertain());
		EXPECT_TRUE(solution->frame()->begin.has_value());
		EXPECT_TRUE(solution->frame()->end.has_value());
		if (solution->frame()->begin.has_value())
			EXPECT_EQ(solution->frame()->begin.value(), 5.0);
		if (solution->frame()->end.has_value())
			EXPECT_EQ(solution->frame()->end.value(), 10.0);
	}
	// no solution because statement only known to be true until 10.0
	statement.setEnd(20.0);
	EXPECT_EQ(lookup(statement).size(), 0);
	// but temporal overlap is sufficient if "sometimes" operator is used
	statement.setTemporalOperator(TemporalOperator::SOMETIMES);
	EXPECT_EQ(lookup(statement).size(), 1);
}

TEST_F(MongoKnowledgeGraphTest, ExtendsTimeInterval) {
	// assert a statement with time interval [10,20]
	FramedTripleCopy statement(swrl_test_"Rex", swrl_test_"hasName", "Rex");
	statement.setBegin(10.0);
	statement.setEnd(20.0);
	EXPECT_EQ(lookup(statement).size(), 0);
	EXPECT_NO_THROW(kg_->insertOne(statement));
	EXPECT_EQ(lookup(statement).size(), 1);
	for (const auto &solution: lookup(statement)) {
		EXPECT_TRUE(solution->frame()->begin.has_value());
		EXPECT_TRUE(solution->frame()->end.has_value());
		if (solution->frame()->begin.has_value())
			EXPECT_EQ(solution->frame()->begin.value(), 5.0);
		if (solution->frame()->end.has_value())
			EXPECT_EQ(solution->frame()->end.value(), 20.0);
	}
	// time interval was merged with existing one into [5,20]
	statement.setBegin(5.0);
	EXPECT_EQ(lookup(statement).size(), 1);
	// no solution because statement only known to be true since 5.0
	statement.setBegin(0.0);
	EXPECT_EQ(lookup(statement).size(), 0);
	// temporal overlap is sufficient if "sometimes" operator is used
	statement.setTemporalOperator(TemporalOperator::SOMETIMES);
	EXPECT_EQ(lookup(statement).size(), 1);
}
