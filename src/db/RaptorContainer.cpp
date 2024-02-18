/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/db/RaptorContainer.h"
#include "knowrob/semweb/xsd.h"

using namespace knowrob;

RDFType getLiteralTypeFromURI(const char *typeURI) {
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

RaptorContainer::RaptorContainer(uint32_t size, std::string_view origin)
		: raptorData_(size),
		  mappedData_(size),
		  origin_(origin),
		  actualSize_(0) {
}

RaptorContainer::RaptorContainer(uint32_t size)
		: raptorData_(size),
		  mappedData_(size),
		  actualSize_(0) {
}

RaptorContainer::~RaptorContainer() {
	reset();
}

void RaptorContainer::reset() {
	for (uint32_t i = 0; i < actualSize_; i++) {
		raptor_free_term(raptorData_[i].s);
		raptor_free_term(raptorData_[i].p);
		raptor_free_term(raptorData_[i].o);
	}
	actualSize_ = 0;
}

StatementData *RaptorContainer::add(raptor_term *s, raptor_term *p, raptor_term *o, librdf_node *context) {
	// validate input from raptor
	if (!s || !p || !o) {
		KB_WARN("received malformed data from raptor, skipping statement.");
		return nullptr;
	}

	// keep a reference on the data
	auto c_s = raptor_term_copy(s);
	auto c_p = raptor_term_copy(p);
	auto c_o = raptor_term_copy(o);
	auto &endpoint_triple = raptorData_[actualSize_];
	endpoint_triple.s = c_s;
	endpoint_triple.p = c_p;
	endpoint_triple.o = c_o;

	// map statement to KnowRob datatype
	auto triple = &mappedData_[actualSize_];

	// read graph name
	if (context) {
		// the context node is a string literal with the graph name
		triple->graph = (const char *) librdf_node_get_literal_value(context);
	} else if (origin_.has_value()) {
		// user specified origin
		triple->graph = origin_.value().data();
	} else {
		triple->graph = nullptr;
	}

	// read predicate
	triple->predicate = (const char *) raptor_uri_as_string(c_p->value.uri);
	// read subject
	if (c_s->type == RAPTOR_TERM_TYPE_BLANK)
		triple->subject = (const char *) c_s->value.blank.string;
	else
		triple->subject = (const char *) raptor_uri_as_string(c_s->value.uri);

	// read object
	if (c_o->type == RAPTOR_TERM_TYPE_BLANK) {
		triple->object = (const char *) c_o->value.blank.string;
		triple->objectType = RDF_RESOURCE;
	} else if (c_o->type == RAPTOR_TERM_TYPE_LITERAL) {
		triple->object = (const char *) c_o->value.literal.string;
		// parse literal type
		if (c_o->value.literal.datatype) {
			auto typeURI = (const char *) raptor_uri_as_string(c_o->value.literal.datatype);
			triple->objectType = getLiteralTypeFromURI(typeURI);
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
		triple->object = (const char *) raptor_uri_as_string(c_o->value.uri);
		triple->objectType = RDF_RESOURCE;
	}

	actualSize_ += 1;
	return triple;
}

StatementData *RaptorContainer::add(raptor_statement *statement, librdf_node *context) {
	return add(statement->subject, statement->predicate, statement->object, context);
}

void RaptorContainer::rollbackLast() {
	if (actualSize_ > 0) {
		actualSize_ -= 1;
		raptor_free_term(raptorData_[actualSize_].s);
		raptor_free_term(raptorData_[actualSize_].p);
		raptor_free_term(raptorData_[actualSize_].o);
	}
}

void RaptorContainer::shrink() {
	mappedData_.resize(actualSize_);
	raptorData_.resize(actualSize_);
}
