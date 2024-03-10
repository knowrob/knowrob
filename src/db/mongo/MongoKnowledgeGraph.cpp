/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/db/mongo/MongoKnowledgeGraph.h"
#include "knowrob/db/mongo/MongoTriplePattern.h"
#include "knowrob/db/mongo/TripleCursor.h"
#include "knowrob/db/mongo/MongoTriple.h"
#include "knowrob/db/mongo/MongoInterface.h"
#include "knowrob/db/BackendManager.h"
#include "knowrob/triples/GraphSequence.h"
#include "knowrob/semweb/rdfs.h"
#include <boost/foreach.hpp>

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

bool MongoKnowledgeGraph::initializeBackend(std::string_view db_uri, std::string_view db_name,
											std::string_view collectionName) {
	tripleCollection_ = connect(db_uri, db_name, collectionName);
	if (tripleCollection_) {
		initializeMongo();
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
			initializeMongo();
			dropSessionOrigins();
			return true;
		} else {
			return false;
		}
	}
	tripleCollection_ = connect(*ptree);
	initializeMongo();

	// set isReadOnly_ flag
	auto o_readOnly = ptree->get_optional<bool>(MONGO_KG_SETTING_READ_ONLY);
	if (o_readOnly.has_value()) {
		isReadOnly_ = o_readOnly.value();
	}
	if (!isReadOnly_) {
		dropSessionOrigins();
	}

	// Auto-drop some named graphs
	auto o_drop_graphs = ptree->get_child_optional(MONGO_KG_SETTING_DROP_GRAPHS);
	if (o_drop_graphs.has_value()) {
		BOOST_FOREACH(const auto &v, o_drop_graphs.value()) {
						removeAllWithOrigin(v.second.data());
					}
	}

	return true;
}

void MongoKnowledgeGraph::initializeMongo() {
	// make sure s/p/o index is defined
	tripleCollection_->createTripleIndex();
	// a collection with just a single document used for querying
	oneCollection_ = std::make_shared<Collection>(
			tripleCollection_->connection(),
			tripleCollection_->dbName().c_str(),
			MONGO_KG_ONE_COLLECTION);
	// Make sure there is one document in the "one" collection.
	// The collection is used to initiate a pipeline for a single input document.
	if (oneCollection_->empty()) {
		Document oneDoc(bson_new());
		bson_t scopeDoc, timeDoc;
		bson_decimal128_t infinity, zero;
		bson_decimal128_from_string(BSON_DECIMAL128_INF, &infinity);
		bson_decimal128_from_string("0", &zero);
		// TODO: time interval is optional now, right? so we do not need to add the fields here? test it!
		//        but maybe mongolog still needs it
		BSON_APPEND_DOCUMENT_BEGIN(oneDoc.bson(), "v_scope", &scopeDoc);
		BSON_APPEND_DOCUMENT_BEGIN(&scopeDoc, "time", &timeDoc);
		BSON_APPEND_DECIMAL128(&timeDoc, "since", &zero);
		BSON_APPEND_DECIMAL128(&timeDoc, "until", &infinity);
		bson_append_document_end(&scopeDoc, &timeDoc);
		bson_append_document_end(oneDoc.bson(), &scopeDoc);
		oneCollection_->storeOne(oneDoc);
	}
	// Create an object used for taxonomy operations
	taxonomy_ = std::make_shared<MongoTaxonomy>(tripleCollection_, oneCollection_);
}

void MongoKnowledgeGraph::dropSessionOrigins() {
	// TODO: move into KnowledgeBase, and do it for all backends
	dropOrigin(semweb::ImportHierarchy::ORIGIN_USER);
	dropOrigin(semweb::ImportHierarchy::ORIGIN_REASONER);
	dropOrigin(semweb::ImportHierarchy::ORIGIN_SESSION);
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

void MongoKnowledgeGraph::drop() {
	tripleCollection_->drop();
	vocabulary_ = std::make_shared<semweb::Vocabulary>();
	importHierarchy_->clear();
}

bool MongoKnowledgeGraph::insertOne(const FramedTriple &tripleData) {
	auto &fallbackOrigin = importHierarchy_->defaultGraph();
	bool isTaxonomic = vocabulary_->isTaxonomicProperty(tripleData.predicate());
	MongoTriple mngTriple(vocabulary_, tripleData, fallbackOrigin, isTaxonomic);
	tripleCollection_->storeOne(mngTriple.document());

	if (semweb::isSubClassOfIRI(tripleData.predicate())) {
		taxonomy_->update({{tripleData.subject(), tripleData.valueAsString()}}, {});
	} else if (semweb::isSubPropertyOfIRI(tripleData.predicate())) {
		taxonomy_->update({}, {{tripleData.subject(), tripleData.valueAsString()}});
	}
	// TODO: It is questionable if this should be handled here, if it should be done at all, or if it rather should be done centrally.
	updateTimeInterval(tripleData);
	return true;
}

bool MongoKnowledgeGraph::insertAll(const semweb::TripleContainerPtr &triples) {
	// only used in case triples do not specify origin field
	auto &fallbackOrigin = importHierarchy_->defaultGraph();
	auto bulk = tripleCollection_->createBulkOperation();
	struct TaxonomyAssertions {
		std::vector<MongoTaxonomy::StringPair> subClassAssertions;
		std::vector<MongoTaxonomy::StringPair> subPropertyAssertions;
	} tAssertions;

	std::for_each(triples->begin(), triples->end(),
				  [&](auto &data) {
					  MongoTriple mngTriple(vocabulary_, *data, fallbackOrigin,
											vocabulary_->isTaxonomicProperty(data->predicate().data()));
					  bulk->pushInsert(mngTriple.document().bson());

					  if (semweb::isSubClassOfIRI(data->predicate())) {
						  tAssertions.subClassAssertions.emplace_back(data->subject(), data->valueAsString());
					  } else if (semweb::isSubPropertyOfIRI(data->predicate())) {
						  tAssertions.subPropertyAssertions.emplace_back(data->subject(), data->valueAsString());
					  }
				  });
	bulk->execute();

	taxonomy_->update(tAssertions.subClassAssertions, tAssertions.subPropertyAssertions);
	for (auto &data: *triples) updateTimeInterval(*data);

	return true;
}

bool MongoKnowledgeGraph::removeOne(const FramedTriple &triple) {
	MongoTriplePattern mngQuery(
			FramedTriplePattern(triple),
			vocabulary_->isTaxonomicProperty(triple.predicate()),
			importHierarchy_);
	tripleCollection_->removeOne(mngQuery.document());
	return true;
}

bool MongoKnowledgeGraph::removeAll(const semweb::TripleContainerPtr &triples) {
	auto bulk = tripleCollection_->createBulkOperation();
	std::for_each(triples->begin(), triples->end(),
				  [this, bulk](auto &data) {
					  MongoTriplePattern mngQuery(
							  FramedTriplePattern(*data),
					  vocabulary_->isTaxonomicProperty(data->predicate()),
							  importHierarchy_);
					  bulk->pushRemoveOne(mngQuery.bson());
				  });
	bulk->execute();

	// TODO: handle hierarchy updates

	return true;
}

bool MongoKnowledgeGraph::dropOrigin(std::string_view graphName) {
	KB_INFO("[mongodb] dropping triples with origin \"{}\".", graphName);
	tripleCollection_->removeAll(Document(
			BCON_NEW("graph", BCON_UTF8(graphName.data()))));
	return true;
}

bool MongoKnowledgeGraph::removeAllWithOrigin(std::string_view graphName) {
	return dropOrigin(graphName);
}

void MongoKnowledgeGraph::count(const ResourceCounter &callback) const {
	for (auto &filename: {PIPELINE_RELATION_COUNTER, PIPELINE_CLASS_COUNTER}) {
		const bson_t *result;
		Cursor cursor(tripleCollection_);
		Document document(Pipeline::loadFromJSON(
				filename, {
						{"COLLECTION", tripleCollection_->name()}
				}));
		cursor.aggregate(document.bson());
		while (cursor.next(&result)) {
			bson_iter_t iter;
			if (!bson_iter_init(&iter, result)) break;
			if (!bson_iter_find(&iter, "resource")) break;
			auto property = bson_iter_utf8(&iter, nullptr);
			if (!bson_iter_find(&iter, "count")) break;
			auto count = bson_iter_as_int64(&iter);
			callback(property, count);
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
	bool b_isTaxonomicProperty;
	if (query.propertyTerm()->termType() == TermType::ATOMIC) {
		b_isTaxonomicProperty = vocabulary_->isTaxonomicProperty(((Atomic *) query.propertyTerm().get())->stringForm());
	} else {
		b_isTaxonomicProperty = false;
	}
	TripleCursor cursor(tripleCollection_);
	// filter documents by triple pattern
	MongoTriplePattern mngQuery(query, b_isTaxonomicProperty, importHierarchy_);
	cursor.filter(mngQuery.bson());
	// iterate over matching documents
	FramedTripleView tripleData;
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

template<typename T>
static inline BindingsCursorPtr doLookup(const T &query, const TripleStore &store) {
	bson_t pipelineDoc = BSON_INITIALIZER;
	bson_t pipelineArray;
	BSON_APPEND_ARRAY_BEGIN(&pipelineDoc, "pipeline", &pipelineArray);
	Pipeline pipeline(&pipelineArray);
	pipeline.append(query, store);
	bson_append_array_end(&pipelineDoc, &pipelineArray);

	auto cursor = std::make_shared<BindingsCursor>(store.oneCollection);
	cursor->aggregate(&pipelineDoc);
	return cursor;
}

BindingsCursorPtr MongoKnowledgeGraph::lookup(const FramedTriplePattern &query) {
	return doLookup(query, mongo::TripleStore(tripleCollection_, oneCollection_, vocabulary_, importHierarchy_));
}

BindingsCursorPtr MongoKnowledgeGraph::lookup(const GraphTerm &query) {
	return doLookup(query, mongo::TripleStore(tripleCollection_, oneCollection_, vocabulary_, importHierarchy_));
}

void MongoKnowledgeGraph::query(const GraphQueryPtr &q, const BindingsHandler &callback) {
	const bool onlyOneSol = (q->ctx()->queryFlags & QUERY_FLAG_ONE_SOLUTION);
	BindingsCursorPtr cursor = lookup(*q->term());
	// NOTE: for some reason below causes a cursor error. looks like a bug in libmongoc to me!
	//if(query->flags() & QUERY_FLAG_ONE_SOLUTION) { cursor->limit(1); }

	while (true) {
		auto next = std::make_shared<Substitution>();
		if (cursor->nextBindings(next)) callback(next);
		else break;
		if (onlyOneSol) break;
	}
}

void MongoKnowledgeGraph::updateTimeInterval(const FramedTriple &tripleData) {
	if (!tripleData.begin().has_value() && !tripleData.end().has_value()) return;
	bool b_isTaxonomicProperty = vocabulary_->isTaxonomicProperty(tripleData.predicate());

	// filter overlapping triples
	TripleCursor cursor(tripleCollection_);
	bson_t selectorDoc = BSON_INITIALIZER;

	FramedTriplePattern overlappingExpr(tripleData);
	overlappingExpr.setIsOccasionalTerm(groundable(Numeric::trueAtom()));

	MongoTriplePattern mngQuery(overlappingExpr, b_isTaxonomicProperty, importHierarchy_);
	cursor.filter(mngQuery.bson());

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
