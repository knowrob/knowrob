/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/knowrob.h"
#include "knowrob/backend/redland/RedlandModel.h"
#include "knowrob/backend/BackendError.h"
#include "knowrob/backend/BackendManager.h"
#include "knowrob/sources/OntologyParser.h"
#include "knowrob/semweb/xsd.h"
#include "knowrob/formulas/Bottom.h"
#include "knowrob/terms/IRIAtom.h"
#include "knowrob/terms/Numeric.h"
#include "knowrob/terms/String.h"
#include "knowrob/terms/Blank.h"
#include "knowrob/queries/AnswerYes.h"

#define REDLAND_SETTING_HOST "host"
#define REDLAND_SETTING_PORT "port"
#define REDLAND_SETTING_USER "user"
#define REDLAND_SETTING_PASSWORD "password"
#define REDLAND_SETTING_DB "db"
#define REDLAND_SETTING_STORAGE "storage"
#define REDLAND_SETTING_ORIGIN "origin"

using namespace knowrob;

#define KNOWROB_RDF_NEW_URI(val) \
    librdf_new_node_from_uri_string(world_, (const unsigned char*)(val).data())
#define KNOWROB_RDF_NEW_BLANK(val) \
    librdf_new_node_from_blank_identifier(world_, (const unsigned char*)(val).data())
#define KNOWROB_RDF_NEW_LITERAL(val, xsdType) \
    librdf_new_node_from_typed_literal(world_, (const unsigned char*)(val), nullptr, xsdType())

/**
 * Register the backend with the BackendManager
 */
KNOWROB_BUILTIN_BACKEND("Redland", RedlandModel)

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

static int logRedlandMessage([[maybe_unused]] void *user_data, librdf_log_message *message) {
	switch (message->level) {
		case LIBRDF_LOG_DEBUG:
			KB_INFO("[redland] {}", librdf_log_message_message(message));
			break;
		case LIBRDF_LOG_INFO:
			KB_INFO("[redland] {}", librdf_log_message_message(message));
			break;
		case LIBRDF_LOG_WARN:
			KB_WARN("[redland] {}", librdf_log_message_message(message));
			break;
		case LIBRDF_LOG_FATAL:
		case LIBRDF_LOG_ERROR:
			KB_ERROR("[redland] {}", librdf_log_message_message(message));
			break;
		default:
			KB_WARN("[redland]: {}", librdf_log_message_message(message));
			break;
	}
	return 1;
}

RedlandModel::RedlandModel()
		: SPARQLBackend(SPARQLFlag::NOT_EXISTS_UNSUPPORTED),
		  world_(nullptr),
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

bool RedlandModel::isPersistent() const {
	return storageType_ != RedlandStorageType::MEMORY &&
		   storageType_ != RedlandStorageType::HASHES;
}

#define REDLAND_REQUIRE_UNINITIALIZED(name, field, val) do {\
    if(isInitialized()) { KB_WARN("attempted to change property {} after initialization.", name); } \
    else { (field) = val; } } while(0)

void RedlandModel::setStorageType(RedlandStorageType storageType) {
	REDLAND_REQUIRE_UNINITIALIZED("storage-type", storageType_, storageType);
}

void RedlandModel::setStorageHashType(RedlandHashType hashType) {
	REDLAND_REQUIRE_UNINITIALIZED("hash-type", hashType_, hashType);
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

void RedlandModel::setStorageDirectory(std::string_view dir) {
	REDLAND_REQUIRE_UNINITIALIZED("dir", storageDir_, dir);
}

void RedlandModel::setRaptorWorld(librdf_world *world) {
	REDLAND_REQUIRE_UNINITIALIZED("raptor-world", world_, world);
}

void RedlandModel::setHashesStorage(RedlandHashType hashType, std::string_view dir) {
	setStorageType(RedlandStorageType::HASHES);
	setStorageHashType(hashType);
	if(hashType == RedlandHashType::BDB) {
		setStorageDirectory(dir);
	}
}

void RedlandModel::setMemoryStorage() {
	setStorageType(RedlandStorageType::MEMORY);
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
		librdf_world_set_logger(world_, nullptr, logRedlandMessage);
	}

	// initialize the storage
	storageOptions_ = getStorageOptions();
	const char *storageOptions = storageOptions_.empty() ? nullptr : storageOptions_.c_str();
	const char *storageType = getStorageTypeString(storageType_);
	const char *storageName = origin_.has_value() ? origin_.value().c_str() : "knowrob";
	KB_INFO("[redland] using storage of type \"{}\" with name \"{}\" and options \"{}\".",
			storageType, storageName, storageOptions);
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
	for (int i = 0; i != static_cast<int>(XSDType::LAST); i++) {
		xsdURIs_[i].set(world_, xsdTypeToIRI((XSDType) i));
	}

	return true;
}

bool RedlandModel::initializeBackend(const PropertyTree &config) {
	if (isInitialized()) {
		KB_WARN("RedlandModel already initialized.");
		return false;
	}
	auto ptree = config.ptree();
	auto o_host = ptree->get_optional<std::string>(REDLAND_SETTING_HOST);
	auto o_port = ptree->get_optional<std::string>(REDLAND_SETTING_PORT);
	auto o_user = ptree->get_optional<std::string>(REDLAND_SETTING_USER);
	auto o_password = ptree->get_optional<std::string>(REDLAND_SETTING_PASSWORD);
	auto o_db = ptree->get_optional<std::string>(REDLAND_SETTING_DB);
	auto o_origin = ptree->get_optional<std::string>(REDLAND_SETTING_ORIGIN);
	auto o_storage = ptree->get_optional<std::string>(REDLAND_SETTING_STORAGE);

	if (o_host) host_ = o_host.value();
	if (o_port) port_ = o_port.value();
	if (o_user) user_ = o_user.value();
	if (o_password) password_ = o_password.value();
	if (o_db) database_ = o_db.value();
	if (o_origin) origin_ = o_origin.value();
	if (o_storage) {
		storageType_ = RedlandStorageType::MEMORY;
		if (o_storage.value() == "hashes") {
			storageType_ = RedlandStorageType::HASHES;
		} else if (o_storage.value() == "mysql") {
			storageType_ = RedlandStorageType::MYSQL;
		} else if (o_storage.value() == "postgresql") {
			storageType_ = RedlandStorageType::POSTGRESQL;
		} else if (o_storage.value() == "sqlite") {
			storageType_ = RedlandStorageType::SQLITE;
		} else {
			KB_WARN("Unknown storage type \"{}\", falling back to memory.", o_storage.value());
		}
	}
	// finally call initializeBackend()
	return initializeBackend();
}

std::string RedlandModel::getStorageOptions() const {
	using OptPair_o = std::pair<std::string, std::optional<std::string>>;
	std::vector<std::pair<std::string, std::string>> opts;
	for (auto &option: {
			OptPair_o{"host", host_},
			OptPair_o{"port", port_},
			OptPair_o{"database", database_},
			OptPair_o{"user", user_},
			OptPair_o{"password", password_},
			OptPair_o{"dir", storageDir_}}) {
		if (option.second.has_value()) {
			opts.emplace_back(option.first, option.second.value());
		}
	}
	// "hash-type" is required by "hashes" storage type
	if(hashType_.has_value()) {
		switch (hashType_.value()) {
			case RedlandHashType::MEMORY:
				opts.emplace_back("hash-type", "memory");
				break;
			case RedlandHashType::BDB:
				opts.emplace_back("hash-type", "bdb");
				break;
		}
	}
	// context support currently is required which limits the storage types we can use.
	// but "memory" and "hashes" can do it.
	opts.emplace_back("contexts", "yes");
	std::stringstream ss;
	for (int i = 0; i < opts.size(); i++) {
		ss << opts[i].first << "='" << opts[i].second << '\'';
		if (i < opts.size() - 1) {
			ss << ",";
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
			tripleFormatMimeType(tripleFormat).data(),
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
				return std::make_shared<IRIAtom>((const char *) librdf_uri_as_string(uri));
			}
			break;
		}
		case LIBRDF_NODE_TYPE_BLANK:
			return std::make_shared<Blank>(
					(const char *) librdf_node_get_blank_identifier(node));
		case LIBRDF_NODE_TYPE_LITERAL: {
			auto literal_value = (const char *) librdf_node_get_literal_value(node);
			auto datatype_uri = librdf_node_get_literal_value_datatype_uri(node);
			TermPtr knowrobTerm;
			if (datatype_uri) {
				auto u_uri_str = librdf_uri_to_string(datatype_uri);
				std::string_view uri_str((const char *) u_uri_str);
				if (xsd::isDoubleType(uri_str)) {
					knowrobTerm = std::make_shared<Double>(std::stod(literal_value));
				} else if (xsd::isIntegerType(uri_str)) {
					knowrobTerm = std::make_shared<Integer>(std::stoll(literal_value));
				} else if (xsd::isBooleanType(uri_str)) {
					knowrobTerm = std::make_shared<Integer>(std::string_view(literal_value) == "true");
				}
				free(u_uri_str);
			}
			if (!knowrobTerm) {
				knowrobTerm = std::make_shared<StringView>(literal_value);
			}
			return knowrobTerm;
		}
		default:
			break;
	}
	return nullptr;
}

bool RedlandModel::insertOne(const FramedTriple &knowrobTriple) {
	auto raptorTriple = librdf_new_statement(world_);
	// map the knowrob triple into a raptor triple
	knowrobToRaptor(knowrobTriple, raptorTriple);
	// add the triple together with a context node holding the origin literal
	librdf_model_context_add_statement(model_, getContextNode(knowrobTriple), raptorTriple);
	librdf_free_statement(raptorTriple);
	return true;
}

bool RedlandModel::insertAll(const TripleContainerPtr &triples) {
	// insert all triples into an in-memory model.
	// only after all triples are inserted, the model is transformed and then pushed to the next stage.
	auto raptorTriple = librdf_new_statement(world_);
	for (auto &knowrobTriple: *triples) {
		// map the knowrob triple into a raptor triple
		knowrobToRaptor(*knowrobTriple, raptorTriple);
		librdf_model_context_add_statement(model_, getContextNode(*knowrobTriple), raptorTriple);
	}
	librdf_free_statement(raptorTriple);
	return true;
}

bool RedlandModel::removeOne(const FramedTriple &knowrobTriple) {
	auto raptorTriple = librdf_new_statement(world_);
	// map the knowrob triple into a raptor triple
	knowrobToRaptor(knowrobTriple, raptorTriple);
	auto ret = librdf_model_context_remove_statement(model_, getContextNode(knowrobTriple), raptorTriple);
	librdf_free_statement(raptorTriple);
	return ret==0;
}

bool RedlandModel::removeAll(const TripleContainerPtr &triples) {
	auto raptorTriple = librdf_new_statement(world_);
	bool allRemoved = true;
	for (auto &knowrobTriple: *triples) {
		// map the knowrob triple into a raptor triple
		knowrobToRaptor(*knowrobTriple, raptorTriple);
		auto ret = librdf_model_context_remove_statement(model_, getContextNode(*knowrobTriple), raptorTriple);
		if (ret != 0) {
			allRemoved = false;
		}
	}
	librdf_free_statement(raptorTriple);
	return allRemoved;
}

bool RedlandModel::removeAllWithOrigin(std::string_view origin) {
	auto stream = librdf_model_find_statements_in_context(
			model_, nullptr, getContextNode(origin));
	if (!stream) return false;
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

void RedlandModel::batch(const TripleHandler &callback) const {
	auto triples = std::make_shared<RaptorContainer>(GlobalSettings::batchSize());
	auto contexts = librdf_model_get_contexts(model_);

	while (!librdf_iterator_end(contexts)) {
		auto context = (librdf_node *) librdf_iterator_get_object(contexts);
		auto stream = librdf_model_find_statements_in_context(
				model_, nullptr, context);

		while (!librdf_stream_end(stream)) {
			auto statement = librdf_stream_get_object(stream);
			triples->add(statement->subject, statement->predicate, statement->object, context);
			if (triples->size() >= GlobalSettings::batchSize()) {
				triples->shrink();
				callback(triples);
				triples->reset();
			}
			librdf_stream_next(stream);
		}
		librdf_free_stream(stream);
		librdf_iterator_next(contexts);
	}
	// Clean up
	if (triples->size() > 0) {
		triples->shrink();
		callback(triples);
	}
	librdf_free_iterator(contexts);
}

bool RedlandModel::contains(const FramedTriple &triple) {
	auto raptorTriple = librdf_new_statement(world_);
	knowrobToRaptor(triple, raptorTriple);
	auto result = librdf_model_contains_statement(model_, raptorTriple);
	librdf_free_statement(raptorTriple);
	return result;
}

void RedlandModel::match(const FramedTriplePattern &query, const TripleVisitor &visitor) {
	auto triples = std::make_shared<RaptorContainer>(1);
	auto rdf_query = librdf_new_statement(world_);
	knowrobToRaptor(query, rdf_query);

	auto stream = librdf_model_find_statements(model_, rdf_query);
	while (!librdf_stream_end(stream)) {
		auto rdf_statement = librdf_stream_get_object(stream);
		triples->add(rdf_statement->subject, rdf_statement->predicate, rdf_statement->object);
		auto &triple = *triples->begin()->ptr;

		if (query.objectOperator() != FilterType::EQ) {
			if (query.filter(triple)) visitor(triple);
		} else {
			visitor(triple);
		}
		triples->reset();
		librdf_stream_next(stream);
	}
	librdf_free_stream(stream);
}

bool RedlandModel::sparql(std::string_view queryString, const BindingsHandler &callback) const {
	auto queryObj = librdf_new_query(
			world_,
			QUERY_LANGUAGE_SPARQL.data(),
			nullptr,
			(unsigned char *) queryString.data(),
			nullptr);
	if (!queryObj) {
		KB_WARN("Failed to create query `{}` for model \"{}\".", queryString, storageName_);
		return false;
	}
	auto results = librdf_query_execute(queryObj, model_);
	if (!results) {
		KB_WARN("Failed to execute query `{}` for model \"{}\".", queryString, storageName_);
		librdf_free_query(queryObj);
		return false;
	}
	while (!librdf_query_results_finished(results)) {
		auto bindings = std::make_shared<Bindings>();

		// read bindings
		int bindings_count = librdf_query_results_get_bindings_count(results);
		for (int i = 0; i < bindings_count; ++i) {
			auto name = std::string_view(librdf_query_results_get_binding_name(results, i));
			if (name.empty() || name[0] == '_') {
				// skip variables that start with '_'.
				// afaik, there is no way to prevent these from being returned when "*" is
				// used in select pattern.
				continue;
			}

			auto node = librdf_query_results_get_binding_value(results, i);
			if (!node) {
				// this indicates that a variable appears in select pattern, but it was not grounded
				// in the query. this is not an error, but rather means the grounding was OPTIONAL.
				// we can just skip this variable.
				continue;
			}

			auto knowrobTerm = termFromNode(node);
			if (knowrobTerm) {
				bindings->set(std::make_shared<Variable>(name), knowrobTerm);
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

librdf_node *RedlandModel::getContextNode(std::string_view origin) {
	auto it = contextNodes_.find(origin);
	if (it != contextNodes_.end()) {
		return it->second;
	}
	auto contextNode = librdf_new_node_from_uri_string(
			world_,
			(const unsigned char*)origin.data());
	contextNodes_[std::string(origin)] = contextNode;
	return contextNode;
}

librdf_node *RedlandModel::getContextNode(const FramedTriple &triple) {
	return getContextNode(
			triple.graph() ? triple.graph().value() :
			origin_.has_value() ? origin_.value() :
			ImportHierarchy::ORIGIN_USER);
}

RedlandURI &RedlandModel::xsdURI(XSDType xsdType) {
	return xsdURIs_[static_cast<int>(xsdType)];
}

raptor_term *RedlandModel::knowrobToRaptor(const TermPtr &term) {
	if (!term || !term->isAtomic()) {
		return nullptr;
	}
	auto atomic = std::static_pointer_cast<Atomic>(term);
	if (term->isIRI()) {
		return KNOWROB_RDF_NEW_URI(atomic->stringForm());
	} else if (term->isBlank()) {
		return KNOWROB_RDF_NEW_BLANK(atomic->stringForm());
	} else if (term->isString()) {
		return KNOWROB_RDF_NEW_LITERAL(atomic->stringForm().data(), xsdURI(XSDType::STRING));
	} else if (term->isNumeric()) {
		auto numeric = std::static_pointer_cast<Numeric>(atomic);
		return KNOWROB_RDF_NEW_LITERAL(atomic->stringForm().data(), xsdURI(numeric->xsdType()));
	}
	KB_WARN("Failed to convert term {} to raptor term.", *term);
	return nullptr;
}

void RedlandModel::knowrobToRaptor(const FramedTriplePattern &pat, raptor_statement *raptorTriple) {
	raptor_term *subject, *predicate, *object;
	if (pat.subjectTerm()) {
		subject = knowrobToRaptor(pat.subjectTerm());
		librdf_statement_set_subject(raptorTriple, subject);
	}
	if (pat.propertyTerm()) {
		predicate = knowrobToRaptor(pat.propertyTerm());
		librdf_statement_set_predicate(raptorTriple, predicate);
	}
	if (pat.objectTerm() && pat.objectOperator() == FilterType::EQ) {
		object = knowrobToRaptor(pat.objectTerm());
		librdf_statement_set_object(raptorTriple, object);
	}
}

void RedlandModel::knowrobToRaptor(const FramedTriple &triple, raptor_statement *raptorTriple) {
	raptor_term *subject, *predicate, *object;
	predicate = KNOWROB_RDF_NEW_URI(triple.predicate());
	if (triple.isSubjectBlank()) {
		subject = KNOWROB_RDF_NEW_BLANK(triple.subject());
	} else {
		subject = KNOWROB_RDF_NEW_URI(triple.subject());
	}
	if (triple.isObjectBlank()) {
		object = KNOWROB_RDF_NEW_BLANK(triple.valueAsString());
	} else if (triple.isObjectIRI()) {
		object = KNOWROB_RDF_NEW_URI(triple.valueAsString());
	} else if (triple.xsdType()) {
		switch (triple.xsdType().value()) {
			case XSDType::STRING:
				object = KNOWROB_RDF_NEW_LITERAL(triple.valueAsString().data(), xsdURI(triple.xsdType().value()));
				break;
			default:
				object = KNOWROB_RDF_NEW_LITERAL(triple.createStringValue().c_str(), xsdURI(triple.xsdType().value()));
				break;
		}
	} else {
		object = KNOWROB_RDF_NEW_LITERAL(triple.createStringValue().c_str(), xsdURI(XSDType::STRING));
	}
	librdf_statement_set_subject(raptorTriple, subject);
	librdf_statement_set_predicate(raptorTriple, predicate);
	librdf_statement_set_object(raptorTriple, object);
}
