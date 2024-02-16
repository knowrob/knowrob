/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <filesystem>
#include <utility>
#include "knowrob/db/OntologyParser.h"
#include "knowrob/KnowledgeBaseError.h"
#include "knowrob/semweb/xsd.h"
#include "knowrob/semweb/owl.h"
#include "knowrob/semweb/PrefixRegistry.h"

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

	semweb::PrefixRegistry::get().registerPrefix(
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
	if (currentBatch_) {
		// there are some triples remaining in the current batch
		flush();
	}
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

raptor_parser *OntologyParser::createParser(knowrob::semweb::TripleFormat format) {
	switch (format) {
		case semweb::RDF_XML:
			return raptor_new_parser(world_, NAME_RDFXML.data());
		case semweb::TURTLE:
			return raptor_new_parser(world_, NAME_TURTLE.data());
		case semweb::N_TRIPLES:
			return raptor_new_parser(world_, NAME_NTRIPLES.data());
		case semweb::RDFA:
			return raptor_new_parser(world_, NAME_RDFA.data());
		case semweb::TRIG:
			return raptor_new_parser(world_, NAME_TRIG.data());
		case semweb::GRDDL:
			return raptor_new_parser(world_, NAME_GRDDL.data());
	}
	return nullptr;
}

void OntologyParser::applyFrame(StatementData *triple) {
	if (frame_) {
		if (frame_->confidence.has_value()) {
			triple->confidence = frame_->confidence.value();
		}
		if (frame_->agent.has_value()) {
			triple->agent = frame_->agent.value()->iri().c_str();
		}
		if (frame_->epistemicOperator.has_value()) {
			triple->epistemicOperator = frame_->epistemicOperator.value();
		}
		if (frame_->temporalOperator.has_value()) {
			triple->temporalOperator = frame_->temporalOperator.value();
		}
		if (frame_->begin.has_value()) {
			triple->begin = frame_->begin.value();
		}
		if (frame_->end.has_value()) {
			triple->end = frame_->end.value();
		}
	}
}

void OntologyParser::add(raptor_statement *statement, const semweb::TripleHandler &callback) {
	if (!currentBatch_) {
		if(origin_.empty()) {
			KB_WARN("No origin set for ontology parser, using default.");
			static const auto defaultOrigin = "user";
			currentBatch_ = std::make_shared<Batch>(callback, batchSize_, defaultOrigin);
		} else {
			currentBatch_ = std::make_shared<Batch>(callback, batchSize_, origin_);
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
	if (triple && semweb::owl::imports == triple->predicate) {
		imports_.emplace_back(triple->object);
	}
	// flush if batch is full
	if (currentBatch_->size() >= batchSize_) {
		flush();
	}
}

void OntologyParser::flush() {
	if (!currentBatch_) return;
	// reduce vector size to actual number of elements
	currentBatch_->shrink();
	// call the callback with the current batch, note that the callback
	// holds a reference on the batch which is only deleted after the callback
	// lifts the reference.
	// TODO: this might block the parser, but we could continue already filling the next batch
	//       on the other hand, if the callback is executed by a worker, maybe too many batches will be queued.
	//KB_DEBUG("flushing {} triples with origin {}", currentBatch_->size(), currentBatch_->origin());
	currentBatch_->callback(currentBatch_);
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

	return (doParse_() == 0);
}

RDFType OntologyParser::getLiteralTypeFromURI(const char *typeURI) {
	if (!typeURI || xsd::isStringType(typeURI))
		return RDF_STRING_LITERAL;
	else if (xsd::isIntegerType(typeURI))
		return RDF_INT64_LITERAL;
	else if (xsd::isDoubleType(typeURI))
		return RDF_DOUBLE_LITERAL;
	else if (xsd::isBooleanType(typeURI))
		return RDF_BOOLEAN_LITERAL;
	else {
		KB_WARN("Unknown data type {} treated as string.", typeURI);
		return RDF_STRING_LITERAL;
	}
}


OntologyParser::Batch::Batch(semweb::TripleHandler callback, uint32_t size, std::string_view origin)
		: callback(std::move(callback)),
		  raptorData_(size),
		  mappedData_(size),
		  origin_(origin),
		  actualSize_(0) {
}

OntologyParser::Batch::~Batch() {
	for (uint32_t i = 0; i < actualSize_; i++) {
		raptor_free_statement(raptorData_[i]);
	}
}

StatementData *OntologyParser::Batch::add(raptor_statement *statement) {
	// validate input from raptor
	if (!statement->subject || !statement->predicate || !statement->object) {
		KB_WARN("received malformed data from raptor, skipping statement.");
		return nullptr;
	}

	auto c_statement = raptor_statement_copy(statement);
	raptorData_[actualSize_] = c_statement;

	// map statement to KnowRob datatype
	auto triple = &mappedData_[actualSize_];
	triple->graph = origin_.data();

	// read predicate
	triple->predicate = (const char *) raptor_uri_as_string(statement->predicate->value.uri);
	// read subject
	if (statement->subject->type == RAPTOR_TERM_TYPE_BLANK)
		triple->subject = (const char *) statement->subject->value.blank.string;
	else
		triple->subject = (const char *) raptor_uri_as_string(statement->subject->value.uri);

	// read object
	if (statement->object->type == RAPTOR_TERM_TYPE_BLANK) {
		triple->object = (const char *) statement->object->value.blank.string;
		triple->objectType = RDF_RESOURCE;
	} else if (statement->object->type == RAPTOR_TERM_TYPE_LITERAL) {
		triple->object = (const char *) statement->object->value.literal.string;
		// parse literal type
		if (statement->object->value.literal.datatype) {
			triple->objectType = getLiteralTypeFromURI((const char *)
															   raptor_uri_as_string(
																	   statement->object->value.literal.datatype));
		} else {
			triple->objectType = RDF_STRING_LITERAL;
		}
		switch (triple->objectType) {
			case RDF_RESOURCE:
			case RDF_STRING_LITERAL:
				break;
			case RDF_DOUBLE_LITERAL:
				triple->objectDouble = strtod(triple->object, nullptr);
				break;
			case RDF_INT64_LITERAL:
				triple->objectInteger = strtol(triple->object, nullptr, 10);
				break;
			case RDF_BOOLEAN_LITERAL: {
				bool objectValue;
				std::istringstream(triple->object) >> objectValue;
				triple->objectInteger = objectValue;
				break;
			}
		}
	} else {
		triple->object = (const char *) raptor_uri_as_string(statement->object->value.uri);
		triple->objectType = RDF_RESOURCE;
	}

	actualSize_ += 1;
	return triple;
}

void OntologyParser::Batch::rollbackLast()
{
	if (actualSize_ > 0) {
		actualSize_ -= 1;
		raptor_free_statement(raptorData_[actualSize_]);
	}
}

void OntologyParser::Batch::shrink() {
	mappedData_.resize(actualSize_);
	raptorData_.resize(actualSize_);
}
