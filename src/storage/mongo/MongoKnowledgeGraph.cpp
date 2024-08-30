/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/storage/mongo/MongoKnowledgeGraph.h"
#include "knowrob/storage/mongo/MongoTriplePattern.h"
#include "knowrob/storage/mongo/TripleCursor.h"
#include "knowrob/storage/mongo/MongoTriple.h"
#include "knowrob/storage/mongo/MongoInterface.h"
#include "knowrob/storage/StorageManager.h"
#include "knowrob/triples/GraphSequence.h"
#include "knowrob/semweb/rdfs.h"
#include "knowrob/knowrob.h"
#include <boost/foreach.hpp>

#define MONGO_KG_ONE_COLLECTION "one"

#define MONGO_KG_SETTING_HOST "host"
#define MONGO_KG_SETTING_PORT "port"
#define MONGO_KG_SETTING_USER "user"
#define MONGO_KG_SETTING_PASSWORD "password"
#define MONGO_KG_SETTING_DB "db"
#define MONGO_KG_SETTING_COLLECTION "collection"
#define MONGO_KG_SETTING_READ_ONLY "read-only"
#define MONGO_KG_SETTING_DROP "drop"

#define MONGO_KG_DEFAULT_HOST "localhost"
#define MONGO_KG_DEFAULT_PORT "27017"
#define MONGO_KG_DEFAULT_DB "knowrob"
#define MONGO_KG_DEFAULT_COLLECTION "triples"

#define PIPELINE_RELATION_COUNTER "storage/mongo/aggregation/relation-counter.json"
#define PIPELINE_CLASS_COUNTER "storage/mongo/aggregation/class-counter.json"

using namespace knowrob;
using namespace knowrob::mongo;
using namespace knowrob::semweb;

/**
 * Register the backend with the BackendManager
 */
KNOWROB_BUILTIN_STORAGE("MongoDB", MongoKnowledgeGraph)

const std::string MongoKnowledgeGraph::DB_URI_DEFAULT = "mongodb://localhost:27017";
const std::string MongoKnowledgeGraph::DB_NAME_KNOWROB = "knowrob";
const std::string MongoKnowledgeGraph::DB_NAME_TESTS = "knowrob_test";
const std::string MongoKnowledgeGraph::COLL_NAME_TRIPLES = "triples";
const std::string MongoKnowledgeGraph::COLL_NAME_TESTS = "triples_test";

static inline StorageFeatures mongoBackendFeatures() {
	return StorageFeature::ReAssignment |
		   StorageFeature::TripleContext;
}

MongoKnowledgeGraph::MongoKnowledgeGraph()
		: QueryableStorage(mongoBackendFeatures()),
		  isReadOnly_(false) {
}

MongoKnowledgeGraph::ConnectionRAII::ConnectionRAII(const MongoKnowledgeGraph *kg)
		: kg(kg), mongo(kg->acquireStore()) {}

MongoKnowledgeGraph::ConnectionRAII::~ConnectionRAII() {
	kg->releaseStore(mongo);
}

mongo::TripleStore MongoKnowledgeGraph::acquireStore() const {
	// Note: We cannot use a Collection object in different threads, so instead we manage
	// a list of active connections and hand them out to threads that need them.
	// If no connection is available, a new one is created on the fly.
	std::lock_guard<std::mutex> lock(storeMutex_);
	if (connections_.empty()) {
		auto tripleCollection = std::make_shared<Collection>(*tripleCollection_);
		auto oneCollection = std::make_shared<Collection>(
				tripleCollection->connection(),
				tripleCollection->dbName().c_str(),
				MONGO_KG_ONE_COLLECTION);
		return {tripleCollection, oneCollection, vocabulary_};
	} else {
		auto store = connections_.front();
		connections_.pop_front();
		return store;
	}
}

void MongoKnowledgeGraph::releaseStore(mongo::TripleStore &store) const {
	std::lock_guard<std::mutex> lock(storeMutex_);
	connections_.push_back(store);
}

bool MongoKnowledgeGraph::initializeBackend(std::string_view db_uri, std::string_view db_name,
											std::string_view collectionName) {
	auto tripleCollection = connect(db_uri, db_name, collectionName);
	if (tripleCollection) {
		initializeMongo(tripleCollection);
		dropSessionOrigins();
		return true;
	} else {
		return false;
	}
}

bool MongoKnowledgeGraph::initializeBackend(const PropertyTree &config) {
	auto ptree = config.ptree();
	if (!ptree) {
		auto tripleCollection = connect(DB_URI_DEFAULT, DB_NAME_KNOWROB, COLL_NAME_TESTS);
		if (tripleCollection) {
			initializeMongo(tripleCollection);
			dropSessionOrigins();
			return true;
		} else {
			return false;
		}
	}
	initializeMongo(connect(*ptree));

	// set isReadOnly_ flag
	auto o_readOnly = ptree->get_optional<bool>(MONGO_KG_SETTING_READ_ONLY);
	if (o_readOnly.has_value()) {
		isReadOnly_ = o_readOnly.value();
	}
	if (!isReadOnly_) {
		dropSessionOrigins();
	}

	// Auto-drop some named graphs
	auto o_drop = ptree->get_child_optional(MONGO_KG_SETTING_DROP);
	if (o_drop.has_value()) {
		if (std::string_view("*") == std::string_view(o_drop.value().data())) {
			drop();
			tripleCollection_->createTripleIndex();
		} else {
			BOOST_FOREACH(const auto &v, o_drop.value()) {
							removeAllWithOrigin(v.second.data());
						}
		}
	}

	return true;
}

void MongoKnowledgeGraph::initializeMongo(const std::shared_ptr<mongo::Collection> &tripleCollection) {
	tripleCollection_ = tripleCollection;
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
		BSON_APPEND_DOCUMENT_BEGIN(oneDoc.bson(), "v_scope", &scopeDoc);
		BSON_APPEND_DOCUMENT_BEGIN(&scopeDoc, "time", &timeDoc);
		BSON_APPEND_DECIMAL128(&timeDoc, "since", &zero);
		BSON_APPEND_DECIMAL128(&timeDoc, "until", &infinity);
		bson_append_document_end(&scopeDoc, &timeDoc);
		bson_append_document_end(oneDoc.bson(), &scopeDoc);
		oneCollection_->storeOne(oneDoc);
	}
	// Create an object used for taxonomy operations
	taxonomy_ = std::make_shared<MongoTaxonomy>(tripleCollection_, oneCollection_, vocabulary_);
	// Add the connection to connection list
	connections_.emplace_back(tripleCollection_, oneCollection_, vocabulary_);
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
	ConnectionRAII scoped(this);
	scoped.mongo.tripleCollection->drop();
	vocabulary_ = std::make_shared<Vocabulary>();
}

bool MongoKnowledgeGraph::insertOne(const FramedTriple &tripleData) {
	ConnectionRAII scoped(this);
	auto &fallbackOrigin = vocabulary_->importHierarchy()->defaultGraph();
	bool isTaxonomic = vocabulary_->isTaxonomicProperty(tripleData.predicate());
	MongoTriple mngTriple(vocabulary_, tripleData, fallbackOrigin, isTaxonomic);
	scoped.mongo.tripleCollection->storeOne(mngTriple.document());

	if (isSubClassOfIRI(tripleData.predicate())) {
		taxonomy_->updateInsert({{tripleData.subject(), tripleData.valueAsString()}}, {});
	} else if (isSubPropertyOfIRI(tripleData.predicate())) {
		taxonomy_->updateInsert({}, {{tripleData.subject(), tripleData.valueAsString()}});
	}

	return true;
}

bool MongoKnowledgeGraph::insertAll(const TripleContainerPtr &triples) {
	ConnectionRAII scoped(this);
	// only used in case triples do not specify origin field
	auto &fallbackOrigin = vocabulary_->importHierarchy()->defaultGraph();
	auto bulk = scoped.mongo.tripleCollection->createBulkOperation();
	std::vector<MongoTaxonomy::StringPair> subClassAssertions;
	std::vector<MongoTaxonomy::StringPair> subPropertyAssertions;

	std::for_each(triples->begin(), triples->end(),
				  [&](auto &data) {
					  MongoTriple mngTriple(vocabulary_, *data, fallbackOrigin,
											vocabulary_->isTaxonomicProperty(data->predicate().data()));
					  bulk->pushInsert(mngTriple.document().bson());

					  if (isSubClassOfIRI(data->predicate())) {
						  subClassAssertions.emplace_back(data->subject(), data->valueAsString());
					  } else if (isSubPropertyOfIRI(data->predicate())) {
						  subPropertyAssertions.emplace_back(data->subject(), data->valueAsString());
					  }
				  });
	bulk->execute();

	taxonomy_->updateInsert(subClassAssertions, subPropertyAssertions);

	return true;
}

bool MongoKnowledgeGraph::removeOne(const FramedTriple &triple) {
	ConnectionRAII scoped(this);
	MongoTriplePattern mngQuery(
			FramedTriplePattern(triple),
			vocabulary_->isTaxonomicProperty(triple.predicate()),
			vocabulary_->importHierarchy());
	scoped.mongo.tripleCollection->removeOne(mngQuery.document());

	if (isSubClassOfIRI(triple.predicate())) {
		taxonomy_->updateRemove({{triple.subject(), triple.valueAsString()}}, {});
	} else if (isSubPropertyOfIRI(triple.predicate())) {
		taxonomy_->updateRemove({}, {{triple.subject(), triple.valueAsString()}});
	}

	return true;
}

bool MongoKnowledgeGraph::removeAll(const TripleContainerPtr &triples) {
	ConnectionRAII scoped(this);
	auto bulk = scoped.mongo.tripleCollection->createBulkOperation();
	std::vector<MongoTaxonomy::StringPair> subClassAssertions;
	std::vector<MongoTaxonomy::StringPair> subPropertyAssertions;

	std::for_each(triples->begin(), triples->end(),
				  [&](auto &data) {
					  MongoTriplePattern mngQuery(
							  FramedTriplePattern(*data),
					  vocabulary_->isTaxonomicProperty(data->predicate()),
							  vocabulary_->importHierarchy());
					  bulk->pushRemoveOne(mngQuery.bson());

					  if (isSubClassOfIRI(data->predicate())) {
						  subClassAssertions.emplace_back(data->subject(), data->valueAsString());
					  } else if (isSubPropertyOfIRI(data->predicate())) {
						  subPropertyAssertions.emplace_back(data->subject(), data->valueAsString());
					  }
				  });
	bulk->execute();

	taxonomy_->updateRemove(subClassAssertions, subPropertyAssertions);

	return true;
}

bool MongoKnowledgeGraph::dropOrigin(std::string_view graphName) {
	KB_DEBUG("[mongodb] dropping triples with origin \"{}\".", graphName);
	ConnectionRAII scoped(this);
	scoped.mongo.tripleCollection->removeAll(Document(
			BCON_NEW("graph", BCON_UTF8(graphName.data()))));
	return true;
}

bool MongoKnowledgeGraph::removeAllWithOrigin(std::string_view graphName) {
	return dropOrigin(graphName);
}

void MongoKnowledgeGraph::count(const ResourceCounter &callback) const {
	ConnectionRAII scoped(this);
	for (auto &filename: {PIPELINE_RELATION_COUNTER, PIPELINE_CLASS_COUNTER}) {
		const bson_t *result;
		Cursor cursor(scoped.mongo.tripleCollection);
		Document document(Pipeline::loadFromJSON(
				filename, {
						{"COLLECTION", scoped.mongo.tripleCollection->name()}
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

void MongoKnowledgeGraph::iterate(TripleCursor &cursor, const TripleVisitor &visitor) {
	FramedTripleView tripleData;
	FramedTriplePtr triplePtr;
	triplePtr.ptr = &tripleData;
	// Mongo cursor own the allocation, and the memory of a document will be deallocated by the cursor during iteration.
	// @see https://mongoc.org/libmongoc/current/mongoc_cursor_next.html
	// So it cannot be allowed that the visitor takes over ownership, hence owned is set to false.
	triplePtr.owned = false;

	// iterate over matching documents
	while (cursor.nextTriple(*triplePtr.ptr)) {
		visitor(triplePtr);
	}
}

void MongoKnowledgeGraph::foreach(const TripleVisitor &visitor) const {
	ConnectionRAII scoped(this);
	TripleCursor cursor(scoped.mongo.tripleCollection);
	iterate(cursor, visitor);
}

static void
batch_(const std::shared_ptr<mongo::Collection> &collection, const TripleHandler &callback, bson_t *filter) {
	TripleCursor cursor(collection);
	if (filter) {
		cursor.filter(filter);
	}
	std::vector<FramedTriplePtr> batchData(GlobalSettings::batchSize());
	uint32_t currentSize = 0;

	while (true) {
		auto &current = batchData[currentSize];
		if (current.ptr && current.owned) {
			delete current.ptr;
		}
		current.ptr = new FramedTripleCopy();
		current.owned = true;

		if (!cursor.nextTriple(*current.ptr)) {
			break;
		}
		currentSize++;
		if (currentSize == GlobalSettings::batchSize()) {
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

void MongoKnowledgeGraph::batch(const TripleHandler &callback) const {
	ConnectionRAII scoped(this);
	batch_(scoped.mongo.tripleCollection, callback, nullptr);
}

void MongoKnowledgeGraph::batchOrigin(std::string_view origin, const TripleHandler &callback) {
	ConnectionRAII scoped(this);
	bson_t filterDoc;
	BSON_APPEND_UTF8(&filterDoc, "graph", origin.data());
	batch_(scoped.mongo.tripleCollection, callback, &filterDoc);
}

void MongoKnowledgeGraph::match(const FramedTriplePattern &query, const TripleVisitor &visitor) {
	ConnectionRAII scoped(this);
	bool b_isTaxonomicProperty;
	if (query.propertyTerm()->termType() == TermType::ATOMIC) {
		b_isTaxonomicProperty = vocabulary_->isTaxonomicProperty(((Atomic *) query.propertyTerm().get())->stringForm());
	} else {
		b_isTaxonomicProperty = false;
	}
	TripleCursor cursor(scoped.mongo.tripleCollection);
	// filter documents by triple pattern
	MongoTriplePattern mngQuery(query, b_isTaxonomicProperty, vocabulary_->importHierarchy());
	cursor.filter(mngQuery.bson());
	iterate(cursor, visitor);
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
	ConnectionRAII scoped(this);
	return doLookup(query, scoped.mongo);
}

BindingsCursorPtr MongoKnowledgeGraph::lookup(const GraphTerm &query) {
	ConnectionRAII scoped(this);
	return doLookup(query, scoped.mongo);
}

void MongoKnowledgeGraph::query(const GraphQueryPtr &q, const BindingsHandler &callback) {
	const bool onlyOneSol = (q->ctx()->queryFlags & QUERY_FLAG_ONE_SOLUTION);
	ConnectionRAII scoped(this);
	BindingsCursorPtr cursor = doLookup(*q->term(), scoped.mongo);
	// NOTE: for some reason below causes a cursor error. looks like a bug in libmongoc to me!
	//if(query->flags() & QUERY_FLAG_ONE_SOLUTION) { cursor->limit(1); }

	while (true) {
		auto next = std::make_shared<Bindings>();
		if (cursor->nextBindings(next)) callback(next);
		else break;
		if (onlyOneSol) break;
	}
}
