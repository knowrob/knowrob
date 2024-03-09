/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

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
#include "knowrob/triples/FramedTriplePattern.h"
#include "knowrob/semweb/rdf.h"
#include "knowrob/semweb/rdfs.h"
#include "knowrob/semweb/owl.h"
#include "knowrob/db/BackendManager.h"
#include "knowrob/KnowledgeBase.h"
#include "knowrob/queries/AnswerNo.h"
#include "knowrob/triples/GraphPattern.h"
#include "knowrob/triples/GraphSequence.h"

#define MONGO_KG_ONE_COLLECTION "one"

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

#define PIPELINE_RELATION_COUNTER "db/mongo/aggregation/relation-counter.json"
#define PIPELINE_CLASS_COUNTER "db/mongo/aggregation/class-counter.json"

using namespace knowrob;
using namespace knowrob::mongo;
using namespace knowrob::semweb;

/**
 * Register the backend with the BackendManager
 */
KNOWROB_BUILTIN_BACKEND("MongoDB", MongoKnowledgeGraph)

const std::string MongoKnowledgeGraph::DB_URI_DEFAULT = "mongodb://localhost:27017";
const std::string MongoKnowledgeGraph::DB_NAME_KNOWROB = "knowrob";
const std::string MongoKnowledgeGraph::DB_NAME_TESTS = "knowrob_test";
const std::string MongoKnowledgeGraph::COLL_NAME_TRIPLES = "triples";
const std::string MongoKnowledgeGraph::COLL_NAME_TESTS = "triples_test";

MongoKnowledgeGraph::MongoKnowledgeGraph()
		: QueryableBackend(),
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

BindingsCursorPtr MongoKnowledgeGraph::lookup(const FramedTriplePattern &tripleExpression) {
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

	auto cursor = std::make_shared<BindingsCursor>(oneCollection_);
	cursor->aggregate(&pipelineDoc);
	return cursor;
}

mongo::BindingsCursorPtr MongoKnowledgeGraph::lookup(const FramedTriple &tripleData) {
	return lookup(FramedTriplePattern(tripleData));
}

mongo::BindingsCursorPtr
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

	auto cursor = std::make_shared<BindingsCursor>(oneCollection_);
	cursor->aggregate(&pipelineDoc);
	return cursor;
}

mongo::BindingsCursorPtr
MongoKnowledgeGraph::lookupSimpleSequence(const std::vector<std::shared_ptr<GraphTerm>> &graphTerms, uint32_t limit) {
	std::vector<FramedTriplePatternPtr> simpleExpressions;
	for (auto &term: graphTerms) {
		if (term->isPattern()) {
			simpleExpressions.push_back(std::static_pointer_cast<GraphPattern>(term)->value());
		}
	}
	return lookup(simpleExpressions, limit);
}

mongo::BindingsCursorPtr MongoKnowledgeGraph::lookupComplex(const GraphQueryPtr &graphQuery, uint32_t limit) {
	KB_WARN("MongoKnowledgeGraph::lookupComplex not implemented yet.");
	// TODO: here it can be assumed that graphQuery has a UNION operation in it, but there is no query
	//       generator for this yet.
	return nullptr;
}

void MongoKnowledgeGraph::count(const ResourceCounter &callback) const {
	{
		const bson_t *result;
		Cursor cursor(tripleCollection_);
		Document document(aggregation::Pipeline::loadFromJSON(
				PIPELINE_RELATION_COUNTER, {
						{"COLLECTION", tripleCollection_->name()}
				}));
		cursor.aggregate(document.bson());
		while (cursor.next(&result)) {
			bson_iter_t iter;
			if (!bson_iter_init(&iter, result)) break;
			if (!bson_iter_find(&iter, "property")) break;
			auto property = bson_iter_utf8(&iter, nullptr);
			if (!bson_iter_find(&iter, "count")) break;
			auto count = bson_iter_as_int64(&iter);
			callback(property, count);
		}
	}
	{
		const bson_t *result;
		Cursor cursor(tripleCollection_);
		Document document(aggregation::Pipeline::loadFromJSON(
				PIPELINE_CLASS_COUNTER, {
						{"COLLECTION", tripleCollection_->name()}
				}));
		cursor.aggregate(document.bson());
		while (cursor.next(&result)) {
			bson_iter_t iter;
			if (!bson_iter_init(&iter, result)) break;
			if (!bson_iter_find(&iter, "class")) break;
			auto cls = bson_iter_utf8(&iter, nullptr);
			if (!bson_iter_find(&iter, "count")) break;
			auto count = bson_iter_as_int64(&iter);
			callback(cls, count);
		}
	}
}

bool MongoKnowledgeGraph::contains(const FramedTriple &triple) {
	bool hasTriple = false;
	match(FramedTriplePattern(triple), [&hasTriple](const FramedTriple &) {
		hasTriple = true;
	});
	return hasTriple;
}

void MongoKnowledgeGraph::foreach(const semweb::TripleVisitor &visitor) const {
	TripleCursor cursor(tripleCollection_);
	FramedTripleView tripleData;
	while (cursor.nextTriple(tripleData)) {
		visitor(tripleData);
	}
}

void MongoKnowledgeGraph::match(const FramedTriplePattern &query, const semweb::TripleVisitor &visitor) {
	bool b_isTaxonomicProperty = isTaxonomicProperty(query.propertyTerm());
	TripleCursor cursor(tripleCollection_);
	FramedTripleView tripleData;
	bson_t selectorDoc = BSON_INITIALIZER;
	// filter triples by query
	aggregation::appendTripleSelector(&selectorDoc,
									  query,
									  b_isTaxonomicProperty,
									  importHierarchy_);
	cursor.filter(&selectorDoc);
	while (cursor.nextTriple(tripleData)) {
		visitor(tripleData);
	}
}

void MongoKnowledgeGraph::batch(const semweb::TripleHandler &callback) const {
	auto batchSize = (batchSize_.has_value() ? batchSize_.value() : 1000);
	TripleCursor cursor(tripleCollection_);
	std::vector<FramedTriplePtr> batchData(batchSize);
	uint32_t currentSize = 0;

	while (true) {
		if (!cursor.nextTriple(*batchData[currentSize].ptr)) {
			break;
		}
		currentSize++;
		if (currentSize == batchSize) {
			auto batch = std::make_shared<ProxyTripleContainer>(&batchData);
			callback(batch);
			currentSize = 0;
		}
	}
	if (currentSize != 0) {
		batchData.resize(currentSize);
		auto batch = std::make_shared<ProxyTripleContainer>(&batchData);
		callback(batch);
	}
}

void MongoKnowledgeGraph::query(const GraphQueryPtr &q, const FramedBindingsHandler &callback) {
	uint32_t limit = (q->ctx()->queryFlags & QUERY_FLAG_ONE_SOLUTION) ? 1 : 0;
	BindingsCursorPtr cursor;

	switch (q->term()->termType()) {
		case GraphTermType::Pattern: {
			auto &pattern = std::static_pointer_cast<GraphPattern>(q->term())->value();
			cursor = lookup({pattern}, limit);
			break;
		}
		case GraphTermType::Sequence: {
			auto &terms = std::static_pointer_cast<GraphSequence>(q->term())->terms();
			bool isSimple = true;
			for (auto &term: terms) {
				if (!term->isPattern()) {
					isSimple = false;
					break;
				}
			}
			if (isSimple) {
				cursor = lookupSimpleSequence(terms, limit);
			} else {
				cursor = lookupComplex(q, limit);
			}
			break;
		}
		case GraphTermType::Union:
			cursor = lookupComplex(q, limit);
			break;
	}
	// NOTE: for some reason below causes a cursor error. looks like a bug in libmongoc to me!
	//       anyways, we add instead a $limit stage in the aggregation pipeline.
	//if(query->flags() & QUERY_FLAG_ONE_SOLUTION) { cursor->limit(1); }

	while (true) {
		auto next = std::make_shared<FramedBindings>();
		if (cursor->nextBindings(next)) {
			callback(next);
		} else {
			break;
		}
	}
}

TokenBufferPtr MongoKnowledgeGraph::watchQuery(const GraphQueryPtr &literal) {
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
	overlappingExpr.setIsOccasionalTerm(groundable(Numeric::trueAtom()));
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

	bool isBelief;
	if (tripleData.confidence().has_value()) {
		BSON_APPEND_DOUBLE(tripleDoc, "confidence", tripleData.confidence().value());
		isBelief = true;
	} else {
		isBelief = tripleData.isUncertain();
	}
	if (isBelief) {
		// flag the statement as "uncertain"
		BSON_APPEND_BOOL(tripleDoc, "uncertain", true);
	}

	if (tripleData.isOccasional()) {
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
