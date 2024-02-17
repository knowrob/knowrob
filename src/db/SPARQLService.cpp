/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/db/SPARQLService.h"
#include "knowrob/db/RaptorContainer.h"
#include "knowrob/Logger.h"
#include "knowrob/KnowledgeBaseError.h"
#include "knowrob/db/OntologyParser.h"

using namespace knowrob;

// The "memory" constant is used as the storage type in librdf.
// This specifies that the RDF data will be stored in memory, which is fast
// but not persistent across different sessions.
static const char *MEMORY_STORAGE = "memory";
static const char *QUERY_LANGUAGE_SPARQL = "sparql";

SPARQLService::SPARQLService(const URI &uri, semweb::TripleFormat format)
		: SPARQLService::SPARQLService(uri, semweb::tripleFormatToString(format)) {
}

SPARQLService::SPARQLService(const URI &uri, std::string_view format)
		: DataService(uri, format),
		  OntologySource(),
		  tripleFormat_(semweb::tripleFormatFromString(format)),
		  world_(nullptr),
		  storage_(nullptr),
		  model_(nullptr) {
	world_ = librdf_new_world();
	librdf_world_open(world_);
	// create a "memory" storage and associate it to the world
	storage_ = librdf_new_storage(
			world_,
			MEMORY_STORAGE,
			nullptr,
			nullptr);
	if (storage_) {
		createModel();
	} else {
		throw KnowledgeBaseError("Failed to create storage for SPARQL endpoint at \"{}\".", uri());
	}
}

SPARQLService::~SPARQLService() {
	librdf_free_model(model_);
	librdf_free_storage(storage_);
	librdf_free_world(world_);
}

bool SPARQLService::createModel() {
	// the name of the model (or NULL)
	static const char *modelName = nullptr;
	// options to initialise model
	//static const char *modelOptionsStr = "contexts=yes&hash-type=md5&write=yes&new=yes";
	static const char *modelOptionsStr = nullptr;

	model_ = librdf_new_model(
			world_,
			storage_,
			modelOptionsStr);
	if (!model_) {
		KB_WARN("Failed to create model of SPARQL endpoint.");
		return false;
	}

	auto rdf_uri = librdf_new_uri(
			world_,
			(const unsigned char *) uri().c_str());
	int returnCode = librdf_model_load(
			model_,
			rdf_uri,
			modelName,
			OntologyParser::mimeType(tripleFormat_),
			nullptr);
	librdf_free_uri(rdf_uri);
	return returnCode == 0;
}

bool SPARQLService::load(const semweb::TripleHandler &callback, std::string_view origin, uint32_t batchSize) {
	static auto query_selectAll = (const unsigned char*) "SELECT ?s ?p ?o WHERE { ?s ?p ?o }";

	auto query = librdf_new_query(
			world_,
			// This is the name of the query language to use (e.g., "sparql").
			QUERY_LANGUAGE_SPARQL,
			// This is the URI identifying the query language. If the name parameter is provided, this can be NULL.
			nullptr,
			// This is the query string to execute.
			query_selectAll,
			// This is the base URI to use for resolving relative URIs in the query string.
			nullptr);
	if(!query) {
		KB_WARN("Failed to create query for SPARQL endpoint at \"{}\".", uri());
		return false;
	}
	auto results = librdf_query_execute(query, model_);
	if (!results) {
		KB_WARN("Failed to execute query for SPARQL endpoint at \"{}\".", uri());
		librdf_free_query(query);
		return false;
	}

	auto batch = std::make_shared<RaptorContainer>(batchSize, origin);
	while (!librdf_query_results_finished(results)) {
		// read bindings
		auto s_term = librdf_query_results_get_binding_value_by_name(results, "s");
		auto p_term = librdf_query_results_get_binding_value_by_name(results, "p");
		auto o_term = librdf_query_results_get_binding_value_by_name(results, "o");
		// process next item
		batch->add(s_term, p_term, o_term);
		if(batch->size() >= batchSize) {
			batch->shrink();
			callback(batch);
			batch->reset();
		}
		// increase iterator
		librdf_query_results_next(results);
	}
	// Clean up
	if(batch->size() > 0) {
		batch->shrink();
		callback(batch);
	}
	librdf_free_query_results(results);
	librdf_free_query(query);
	return true;
}
