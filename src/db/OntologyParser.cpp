/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <filesystem>
#include "knowrob/db/OntologyParser.h"
#include "knowrob/KnowledgeBaseError.h"
#include "knowrob/semweb/owl.h"
#include "knowrob/semweb/PrefixRegistry.h"
#include "knowrob/semweb/ImportHierarchy.h"

namespace fs = std::filesystem;
using namespace knowrob;

struct RaptorUserData {
	OntologyParser *parser;
	semweb::TripleHandler callback;
};

static void processTriple(void *userData, raptor_statement *statement) {
	auto x = (RaptorUserData *) userData;
	x->parser->add(statement, x->callback);
}

static void procesNamespace([[maybe_unused]] void *userData, raptor_namespace *nspace) {
	auto r_prefix = raptor_namespace_get_prefix(nspace);
	if(!r_prefix) return;

	auto r_uri = raptor_namespace_get_uri(nspace);
	if(!r_uri) return;
	auto r_uriString = raptor_uri_as_string(r_uri);

	semweb::PrefixRegistry::registerPrefix(
			std::string_view((const char *) r_prefix),
			std::string_view((const char *) r_uriString));
}

static void raptor_log(void *, raptor_log_message *message) {
	switch (message->level) {
		case RAPTOR_LOG_LEVEL_ERROR:
		case RAPTOR_LOG_LEVEL_FATAL:
			KB_ERROR("[raptor] {}", message->text);
			break;
		case RAPTOR_LOG_LEVEL_WARN:
			KB_WARN("[raptor] {}", message->text);
			break;
		case RAPTOR_LOG_LEVEL_NONE:
		case RAPTOR_LOG_LEVEL_TRACE:
		case RAPTOR_LOG_LEVEL_DEBUG:
		case RAPTOR_LOG_LEVEL_INFO:
			break;
	}
}

// these are all formats supported by raptor according to:
// https://librdf.org/raptor/api-1.4/raptor-parsers.html#raptor-parsers-intro
const std::string OntologyParser::NAME_TURTLE = "turtle";
const std::string OntologyParser::NAME_TRIG = "trig";
const std::string OntologyParser::NAME_GRDDL = "grddl";
const std::string OntologyParser::NAME_NTRIPLES = "ntriples";
const std::string OntologyParser::NAME_RDFXML = "rdfxml";
const std::string OntologyParser::NAME_RDFA = "rdfa";
const std::string OntologyParser::NAME_RSS_TAG_SOUP = "rss-tag-soup";

OntologyParser::OntologyParser(const std::string_view &fileURI, knowrob::semweb::TripleFormat format,
							   uint32_t batchSize)
		: batchSize_(batchSize) {
	world_ = createWorld();
	parser_ = createParser(format);
	// raptor can report namespaces
	raptor_parser_set_namespace_handler(parser_, nullptr, procesNamespace);

	if (fs::exists(fileURI)) {
		// Parse the content of a local file URI
		auto escapedString = raptor_uri_filename_to_uri_string(fileURI.data());
		uri_ = raptor_new_uri(world_, (unsigned char *) escapedString);
		doParse_ = [this]() {
			return raptor_parser_parse_file(parser_, uri_, uriBase_);
		};
		raptor_free_memory(escapedString);
	} else {
		// Parse the content from a remote file URI
		uri_ = raptor_new_uri(world_, (const unsigned char *) fileURI.data());
		doParse_ = [this]() {
			return raptor_parser_parse_uri(parser_, uri_, uriBase_);
		};
	}
	uriBase_ = raptor_uri_copy(uri_);
}

OntologyParser::~OntologyParser() {
	raptor_free_parser(parser_);
	raptor_free_uri(uri_);
	raptor_free_uri(uriBase_);
	raptor_free_world(world_);
}

raptor_world *OntologyParser::createWorld() {
	auto world = raptor_new_world();
	// redirect log messages to the knowrob logger
	raptor_world_set_log_handler(world, nullptr, raptor_log);
	if (raptor_world_open(world) != 0) {
		throw KnowledgeBaseError("failed to initialize raptor library.");
	}
	return world;
}

const char* OntologyParser::mimeType(knowrob::semweb::TripleFormat format) {
	switch (format) {
		case semweb::RDF_XML:
			return NAME_RDFXML.data();
		case semweb::TURTLE:
			return NAME_TURTLE.data();
		case semweb::N_TRIPLES:
			return NAME_NTRIPLES.data();
		case semweb::RDFA:
			return NAME_RDFA.data();
		case semweb::TRIG:
			return NAME_TRIG.data();
		case semweb::GRDDL:
			return NAME_GRDDL.data();
	}
	return nullptr;
}

raptor_parser *OntologyParser::createParser(knowrob::semweb::TripleFormat format) {
	return raptor_new_parser(world_, mimeType(format));
}

void OntologyParser::applyFrame(FramedTriple *triple) {
	if (frame_) {
		if (frame_->confidence.has_value()) {
			triple->setConfidence(frame_->confidence.value());
			triple->setIsUncertain(true);
		}
		if (frame_->perspective.has_value()) {
			triple->setPerspective(frame_->perspective.value()->iri());
		}
		if (frame_->uncertain){
			triple->setIsUncertain(true);
		}
		if (frame_->occasional){
			triple->setIsOccasional(true);
		}
		if (frame_->begin.has_value()) {
			triple->setBegin(frame_->begin.value());
		}
		if (frame_->end.has_value()) {
			triple->setEnd(frame_->end.value());
		}
	}
}

void OntologyParser::add(raptor_statement *statement, const semweb::TripleHandler &callback) {
	if (!currentBatch_) {
		if(origin_.empty()) {
			KB_WARN("No origin set for ontology parser, falling back to \"user\" origin.");
			currentBatch_ = std::make_shared<RaptorContainer>(batchSize_, semweb::ImportHierarchy::ORIGIN_USER);
		} else {
			currentBatch_ = std::make_shared<RaptorContainer>(batchSize_, origin_);
		}
	}
	// add to batch, map into knowrob data structures
	auto triple = currentBatch_->add(statement);
	if(filter_ && !filter_(*triple)) {
		currentBatch_->rollbackLast();
		return;
	}
	applyFrame(triple);
	// remember imports
	if (triple && semweb::owl::imports->stringForm() == triple->predicate()) {
		imports_.emplace_back(triple->valueAsString());
	}
	// flush if batch is full
	if (currentBatch_->size() >= batchSize_) {
		flush(callback);
	}
}

void OntologyParser::flush(const semweb::TripleHandler &callback) {
	if (!currentBatch_) return;
	// reduce vector size to actual number of elements
	currentBatch_->shrink();
	// call the callback with the current batch, note that the callback
	// holds a reference on the batch which is only deleted after the callback
	// lifts the reference.
	// TODO: this might block the parser, but we could continue already filling the next batch
	//       on the other hand, if the callback is executed by a worker, maybe too many batches will be queued.
	//       but we could have a max num of batches in the queue, and use join to wait for the a worker to be done if too many.
	//KB_DEBUG("flushing {} triples with origin {}", currentBatch_->size(), currentBatch_->origin());
	callback(currentBatch_);
	currentBatch_ = nullptr;
}


bool OntologyParser::run(const semweb::TripleHandler &callback) {
	// call processTriple for each loaded triple
	RaptorUserData userData = {const_cast<OntologyParser *>(this), callback};
	raptor_parser_set_statement_handler(parser_, &userData, processTriple);

	// make sure blanks are generated with proper prefix.
	raptor_world_set_generate_bnodeid_parameters(
			raptor_parser_get_world(parser_),
			blankPrefix_.data(), 1);

	auto exit_status = doParse_();
	if(exit_status == 0) {
		flush(callback);
	} else {
		currentBatch_ = nullptr;
	}
	return (exit_status == 0);
}
