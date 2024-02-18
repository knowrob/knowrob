/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/db/RedlandModel.h"
#include "knowrob/db/BackendError.h"
#include "knowrob/db/OntologyParser.h"
#include "knowrob/semweb/xsd.h"
#include "knowrob/formulas/Bottom.h"

using namespace knowrob;

#define KNOWROB_RDF_NEW_URI(val) \
    librdf_new_node_from_uri_string(world_, (const unsigned char*)val)
#define KNOWROB_RDF_NEW_LITERAL(val, xsdType) \
    librdf_new_node_from_typed_literal(world_, (const unsigned char*)val, nullptr, xsdType)

static inline const char *getStorageTypeString(RedlandStorageType storageType) {
	switch (storageType) {
		case RedlandStorageType::MEMORY:
			return "memory";
		case RedlandStorageType::HASHES:
			return "hashes";
		case RedlandStorageType::MYSQL:
			return "mysql";
		case RedlandStorageType::POSTGRESQL:
			return "postgresql";
		case RedlandStorageType::SQLITE:
			return "sqlite";
	}
	return "unknown";
}

RedlandModel::RedlandModel()
		: world_(nullptr),
		  ownedWorld_(nullptr),
		  model_(nullptr),
		  storage_(nullptr),
		  storageType_(RedlandStorageType::MEMORY) {
}

RedlandModel::~RedlandModel() {
	finalize();
}

void RedlandModel::finalize() {
	for (auto &pair: contextNodes_) {
		librdf_free_node(pair.second);
	}
	contextNodes_.clear();

	if (model_) {
		librdf_free_model(model_);
		model_ = nullptr;
	}
	if (storage_) {
		librdf_free_storage(storage_);
		storage_ = nullptr;
	}
	if (ownedWorld_) {
		librdf_free_world(ownedWorld_);
		ownedWorld_ = nullptr;
		world_ = nullptr;
	}
}

bool RedlandModel::isInitialized() const {
	return model_ != nullptr;
}

#define REDLAND_REQUIRE_UNINITIALIZED(name, field, val) \
    if(isInitialized()) { KB_WARN("attempted to change property {} after initialization.", name); } \
    else field = val

void RedlandModel::setStorageType(RedlandStorageType storageType) {
	REDLAND_REQUIRE_UNINITIALIZED("storage-type", storageType_, storageType);
}

void RedlandModel::setHost(std::string_view host) {
	REDLAND_REQUIRE_UNINITIALIZED("host", host_, host);
}

void RedlandModel::setDatabase(std::string_view database) {
	REDLAND_REQUIRE_UNINITIALIZED("database", database_, database);
}

void RedlandModel::setUser(std::string_view user) {
	REDLAND_REQUIRE_UNINITIALIZED("user", user_, user);
}

void RedlandModel::setPassword(std::string_view password) {
	REDLAND_REQUIRE_UNINITIALIZED("password", password_, password);
}

void RedlandModel::setRaptorWorld(librdf_world *world) {
	REDLAND_REQUIRE_UNINITIALIZED("raptor-world", world_, world);
}

bool RedlandModel::initializeBackend() {
	if (isInitialized()) {
		KB_WARN("RedlandModel already initialized.");
		return false;
	}

	// initialize the world if it hasn't been set before
	if (!world_) {
		ownedWorld_ = world_ = librdf_new_world();
		if (!world_) {
			throw BackendError("Failed to create Redland world.");
		}
		librdf_world_open(world_);
	}

	// initialize the storage
	storageOptions_ = getStorageOptions();
	const char *storageOptions = storageOptions_.empty() ? nullptr : storageOptions_.c_str();
	const char *storageType = getStorageTypeString(storageType_);
	const char *storageName = origin_.has_value() ? origin_.value().c_str() : "knowrob";
	storage_ = librdf_new_storage(world_,
								  storageType,
								  storageName,
								  storageOptions);
	if (!storage_) {
		throw BackendError("Failed to create Redland storage for type \"{}\".", storageType);
	}

	// initialize the model
	static const char *modelOptions = nullptr;
	model_ = librdf_new_model(world_, storage_, modelOptions);
	if (!model_) {
		librdf_free_storage(storage_);
		storage_ = nullptr;
		throw BackendError("Failed to create Redland model for storage type \"{}\".", storageType);
	}

	// initialize URIs
	uri_xsdBoolean_.set(world_, xsd::IRI_boolean);
	uri_xsdInteger_.set(world_, xsd::IRI_integer);
	uri_xsdDouble_.set(world_, xsd::IRI_double);
	uri_xsdString_.set(world_, xsd::IRI_string);

	return true;
}

bool RedlandModel::initializeBackend(const ReasonerConfig &config) {
	if (isInitialized()) {
		KB_WARN("RedlandModel already initialized.");
		return false;
	}
	// read options from config
	// TODO: implement
	KB_WARN("RedlandModel::initializeBackend(config) not implemented.");
	// finally call initializeBackend()
	return initializeBackend();
}

std::string RedlandModel::getStorageOptions() const {
	using OptPair_o = std::pair<std::string, std::optional<std::string>>;
	std::vector<std::pair<std::string, std::string>> opts;
	for (auto &option: {
			OptPair_o{"host", host_},
			OptPair_o{"database", database_},
			OptPair_o{"user", user_},
			OptPair_o{"password", password_}}) {
		if (option.second.has_value()) {
			opts.emplace_back(option.first, option.second.value());
		}
	}
	std::stringstream ss;
	for (int i = 0; i < opts.size(); i++) {
		ss << opts[i].first << "='" << opts[i].second;
		if (i < opts.size() - 1) {
			ss << "',";
		}
	}
	return ss.str();
}

bool RedlandModel::load(const URI &uri, semweb::TripleFormat tripleFormat) {
	static const char *modelName = nullptr;
	auto rdf_uri = librdf_new_uri(
			world_,
			(const unsigned char *) uri().c_str());
	int returnCode = librdf_model_load(
			model_,
			rdf_uri,
			modelName,
			OntologyParser::mimeType(tripleFormat), // TODO move mimeType def
			nullptr);
	librdf_free_uri(rdf_uri);
	return returnCode == 0;
}

static inline TermPtr termFromNode(librdf_node *node) {
	if (!node) return nullptr;

	switch (librdf_node_get_type(node)) {
		case LIBRDF_NODE_TYPE_RESOURCE: {
			auto uri = librdf_node_get_uri(node);
			if (uri) {
				return std::make_shared<StringTerm>((const char *) librdf_uri_as_string(uri));
			}
			break;
		}
		case LIBRDF_NODE_TYPE_LITERAL: {
			auto literal_value = (const char *) librdf_node_get_literal_value(node);
			auto datatype_uri = librdf_node_get_literal_value_datatype_uri(node);
			TermPtr knowrobTerm;
			if (datatype_uri) {
				auto u_uri_str = librdf_uri_to_string(datatype_uri);
				std::string_view uri_str((const char *) u_uri_str);
				if (xsd::isDoubleType(uri_str)) {
					knowrobTerm = std::make_shared<DoubleTerm>(std::stod(literal_value));
				} else if (xsd::isIntegerType(uri_str)) {
					knowrobTerm = std::make_shared<Integer32Term>(std::stoll(literal_value));
				} else if (xsd::isBooleanType(uri_str)) {
					knowrobTerm = std::make_shared<Integer32Term>(std::string_view(literal_value) == "true");
				}
				free(u_uri_str);
			}
			if (!knowrobTerm) {
				knowrobTerm = std::make_shared<StringTerm>(literal_value);
			}
			return knowrobTerm;
		}
		case LIBRDF_NODE_TYPE_BLANK:
			return std::make_shared<StringTerm>(
					(const char *) librdf_node_get_blank_identifier(node));
		default:
			break;
	}
	return nullptr;
}

bool RedlandModel::query(const SPARQLQuery &query, const SubstitutionHandler &callback) const {
	// TODO: add query interface to a common base class?
	// TODO: a more simple interface could also provide origin of triple, but we cannot provide this from sparql
	//       in a standard way.

	auto queryObj = librdf_new_query(
			world_,
			QUERY_LANGUAGE_SPARQL.data(),
			nullptr,
			query.asUnsignedString(),
			nullptr);
	if (!queryObj) {
		KB_WARN("Failed to create query `{}` for model \"{}\".", query(), storageName_);
		return false;
	}
	auto results = librdf_query_execute(queryObj, model_);
	if (!results) {
		KB_WARN("Failed to execute query `{}` for model \"{}\".", query(), storageName_);
		librdf_free_query(queryObj);
		return false;
	}
	while (!librdf_query_results_finished(results)) {
		auto bindings = std::make_shared<Substitution>();

		// read bindings
		int bindings_count = librdf_query_results_get_bindings_count(results);
		for (int i = 0; i < bindings_count; ++i) {
			auto name = librdf_query_results_get_binding_name(results, i);
			auto node = librdf_query_results_get_binding_value(results, i);
			auto knowrobTerm = termFromNode(node);
			if (knowrobTerm) {
				bindings->set(Variable(name), knowrobTerm);
			} else {
				KB_WARN("Failed to process binding for variable \"{}\".", name);
			}
			librdf_free_node(node);
		}
		callback(bindings);
		librdf_query_results_next(results);
	}
	librdf_free_query_results(results);
	librdf_free_query(queryObj);
	return true;
}

bool RedlandModel::foreach(const semweb::MutableTripleHandler &callback) const {
	// TODO: add foreach interface to a common base class?
	auto batchSize = (batchSize_.has_value() ? batchSize_.value() : 1000);
	auto batch = std::make_shared<RaptorContainer>(batchSize);
	auto contexts = librdf_model_get_contexts(model_);

	while (!librdf_iterator_end(contexts)) {
		auto context = (librdf_node *) librdf_iterator_get_object(contexts);
		auto stream = librdf_model_find_statements_in_context(
				model_, nullptr, context);

		while (!librdf_stream_end(stream)) {
			auto statement = librdf_stream_get_object(stream);
			// process next item
			batch->add(statement->subject, statement->predicate, statement->object, context);
			if (batch->size() >= batchSize) {
				batch->shrink();
				callback(batch);
				batch->reset();
			}
			librdf_stream_next(stream);
		}
		librdf_free_stream(stream);
		librdf_iterator_next(contexts);
	}
	// Clean up
	if (batch->size() > 0) {
		batch->shrink();
		callback(batch);
	}
	librdf_free_iterator(contexts);
	return true;
}

bool RedlandModel::insertOne(const StatementData &knowrobTriple) {
	auto raptorTriple = librdf_new_statement(world_);
	// map the knowrob triple into a raptor triple
	knowrobToRaptor(knowrobTriple, raptorTriple);
	// add the triple together with a context node holding the origin literal
	librdf_model_context_add_statement(model_, getContextNode(knowrobTriple), raptorTriple);
	librdf_free_statement(raptorTriple);
	return true;
}

bool RedlandModel::insertAll(const semweb::TripleContainerPtr &triples) {
	// insert all triples into an in-memory model.
	// only after all triples are inserted, the model is transformed and then pushed to the next stage.
	// TODO: rather do batch insert, I bet librdf can do something...
	auto raptorTriple = librdf_new_statement(world_);
	for (auto &knowrobTriple: *triples) {
		// map the knowrob triple into a raptor triple
		knowrobToRaptor(knowrobTriple, raptorTriple);
		librdf_model_context_add_statement(model_, getContextNode(knowrobTriple), raptorTriple);
	}
	librdf_free_statement(raptorTriple);
	return true;
}

bool RedlandModel::removeOne(const StatementData &knowrobTriple) {
	auto raptorTriple = librdf_new_statement(world_);
	// map the knowrob triple into a raptor triple
	knowrobToRaptor(knowrobTriple, raptorTriple);
	librdf_model_remove_statement(model_, raptorTriple);
	librdf_free_statement(raptorTriple);
	return true;
}

bool RedlandModel::removeAll(const semweb::TripleContainerPtr &triples) {
	auto raptorTriple = librdf_new_statement(world_);
	for (auto &knowrobTriple: *triples) {
		// map the knowrob triple into a raptor triple
		knowrobToRaptor(knowrobTriple, raptorTriple);
		librdf_model_remove_statement(model_, raptorTriple);
	}
	librdf_free_statement(raptorTriple);
	return true;
}

bool RedlandModel::removeAllWithOrigin(std::string_view origin) {
	auto stream = librdf_model_find_statements_in_context(
			model_, nullptr, getContextNode(origin));
	// collect matching statements
	std::vector<librdf_statement *> toRemove;
	while (!librdf_stream_end(stream)) {
		auto next = librdf_stream_get_object(stream);
		toRemove.push_back(librdf_new_statement_from_statement(next));
		librdf_stream_next(stream);
	}
	librdf_free_stream(stream);
	// finally remove matching statements
	for (auto statement: toRemove) {
		librdf_model_remove_statement(model_, statement);
		librdf_free_statement(statement);
	}
	return true;
}

bool RedlandModel::removeAllMatching(const RDFLiteral &lit) {
	auto instances = std::make_shared<RDFLiteralContainer>();
	query(SPARQLQuery(lit), [&](const SubstitutionPtr &bindings) {
		instances->push_back(std::make_shared<RDFLiteral>(lit, *bindings));
	});
	removeAll(instances);
	return true;
}

librdf_node *RedlandModel::getContextNode(std::string_view origin) {
	auto it = contextNodes_.find(origin);
	if (it != contextNodes_.end()) {
		return it->second;
	}
	auto contextNode = librdf_new_node_from_literal(
			world_,
			(const unsigned char *) origin.data(),
			nullptr,
			0);
	contextNodes_[std::string(origin)] = contextNode;
	return contextNode;
}

librdf_node *RedlandModel::getContextNode(const StatementData &triple) {
	return getContextNode(
			triple.graph ? triple.graph :
			origin_.has_value() ? origin_.value() :
			semweb::ImportHierarchy::ORIGIN_USER);
}

void RedlandModel::knowrobToRaptor(const StatementData &triple, raptor_statement *raptorTriple) {
	auto subject = KNOWROB_RDF_NEW_URI(triple.subject);
	auto predicate = KNOWROB_RDF_NEW_URI(triple.predicate);
	raptor_term *object = nullptr;
	switch (triple.objectType) {
		case RDF_RESOURCE:
			object = KNOWROB_RDF_NEW_URI(triple.object);
			break;
		case RDF_STRING_LITERAL:
			object = KNOWROB_RDF_NEW_LITERAL(triple.object, uri_xsdString_());
			break;
		case RDF_DOUBLE_LITERAL:
			object = KNOWROB_RDF_NEW_LITERAL(triple.object, uri_xsdDouble_());
			break;
		case RDF_INT64_LITERAL:
			object = KNOWROB_RDF_NEW_LITERAL(triple.object, uri_xsdInteger_());
			break;
		case RDF_BOOLEAN_LITERAL:
			object = KNOWROB_RDF_NEW_LITERAL(triple.object, uri_xsdBoolean_());
			break;
	}
	librdf_statement_set_subject(raptorTriple, subject);
	librdf_statement_set_predicate(raptorTriple, predicate);
	librdf_statement_set_object(raptorTriple, object);
}
