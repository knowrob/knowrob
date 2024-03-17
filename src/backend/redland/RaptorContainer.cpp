/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/backend/redland/RaptorContainer.h"
#include "knowrob/terms/XSDAtomic.h"

using namespace knowrob;

RaptorContainer::RaptorContainer(uint32_t size, std::string_view origin)
		: raptorData_(size),
		  mappedData_(size),
		  origin_(origin),
		  actualSize_(0) {
	for (auto &triple: mappedData_) {
		triple.ptr = new FramedTripleView();
		triple.owned = true;
	}
}

RaptorContainer::RaptorContainer(uint32_t size)
		: raptorData_(size),
		  mappedData_(size),
		  actualSize_(0) {
}

RaptorContainer::~RaptorContainer() {
	reset();
}

TripleContainer::ConstGenerator RaptorContainer::cgenerator() const {
	return [this, i = 0]() mutable -> const FramedTriplePtr * {
		if (i < actualSize_) return &mappedData_[i++];
		return nullptr;
	};
}

MutableTripleContainer::MutableGenerator RaptorContainer::generator() {
	return [this, i = 0]() mutable -> FramedTriplePtr * {
		if (i < actualSize_) return &mappedData_[i++];
		return nullptr;
	};
}

void RaptorContainer::reset() {
	for (uint32_t i = 0; i < actualSize_; i++) {
		raptor_free_term(raptorData_[i].s);
		raptor_free_term(raptorData_[i].p);
		raptor_free_term(raptorData_[i].o);
	}
	actualSize_ = 0;
}

FramedTriple *RaptorContainer::add(raptor_term *s, raptor_term *p, raptor_term *o, librdf_node *context) {
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
	auto triple = mappedData_[actualSize_];
	triple->reset();

	// read graph name
	if (context) {
		// the context node is a string literal with the graph name
		triple->setGraph((const char *) librdf_node_get_literal_value(context));
	} else if (origin_.has_value()) {
		// user specified origin
		triple->setGraph(origin_.value().data());
	}

	// read predicate
	triple->setPredicate((const char *) raptor_uri_as_string(c_p->value.uri));
	// read subject
	if (c_s->type == RAPTOR_TERM_TYPE_BLANK)
		triple->setSubjectBlank((const char *) c_s->value.blank.string);
	else
		triple->setSubject((const char *) raptor_uri_as_string(c_s->value.uri));

	// read object
	if (c_o->type == RAPTOR_TERM_TYPE_BLANK) {
		triple->setObjectBlank((const char *) c_o->value.blank.string);
	} else if (c_o->type == RAPTOR_TERM_TYPE_LITERAL) {
		auto stringForm = (const char *) c_o->value.literal.string;
		// parse literal type
		XSDType xsdType = XSDType::STRING;
		if (c_o->value.literal.datatype) {
			auto typeURI = (const char *) raptor_uri_as_string(c_o->value.literal.datatype);
			xsdType = xsdTypeFromIRI(typeURI);
		}
		triple->setXSDValue(stringForm, xsdType);
	} else {
		triple->setObjectIRI((const char *) raptor_uri_as_string(c_o->value.uri));
	}

	actualSize_ += 1;
	return triple.ptr;
}

FramedTriple *RaptorContainer::add(raptor_statement *statement, librdf_node *context) {
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
