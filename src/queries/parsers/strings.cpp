/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <boost/spirit/include/phoenix.hpp>
#include "knowrob/queries/QueryError.h"
#include "knowrob/queries/parsers/strings.h"
#include "knowrob/semweb/PrefixRegistry.h"

static std::string createIRI(const std::string &prefix, const std::string &name) {
	auto uri = knowrob::PrefixRegistry::createIRI(prefix, name);
	if (uri.has_value()) {
		return uri.value();
	} else {
		throw knowrob::QueryError("Cannot construct IRI for '{}': " "IRI prefix '{}' is not registered!", name, prefix);
	}
}

namespace knowrob::parsers::str {
	namespace qi = boost::spirit::qi;
	namespace ascii = boost::spirit::ascii;
	#define RETURN_STRING_RULE(expr) static StringRule r(expr); return r

	StringRule &single_quotes() {
		RETURN_STRING_RULE(qi::lexeme['\'' >> +(qi::char_ - '\'') >> '\'']);
	}

	StringRule &double_quotes() {
		RETURN_STRING_RULE(qi::lexeme['"' >> +(qi::char_ - '"') >> '"']);
	}

	StringRule &lower_prefix() {
		RETURN_STRING_RULE(qi::raw[ascii::lower >> *(ascii::alnum | '_')]);
	}

	StringRule &upper_prefix() {
		RETURN_STRING_RULE(qi::raw[ascii::upper >> *(ascii::alnum | '_')]);
	}

	StringRule &iri_ns() {
		RETURN_STRING_RULE(qi::raw[ascii::alpha >> *(ascii::alnum | '_')]);
	}

	StringRule &iri_entity() {
		RETURN_STRING_RULE(str::single_quotes() | qi::raw[ascii::alpha >> *(ascii::alnum | '_')]);
	}

	StringRule &iri() {
		RETURN_STRING_RULE((iri_ns() >> ':' >> iri_entity())
		                   [qi::_val = boost::phoenix::bind(&createIRI, qi::_1, qi::_2)]);
	}

	StringRule &atom() {
		RETURN_STRING_RULE(single_quotes() | lower_prefix());
	}

	StringRule &atom_or_iri() {
		RETURN_STRING_RULE(iri() | atom());
	}

	StringRule &blank() {
		RETURN_STRING_RULE(qi::raw['_']);
	}

	StringRule &number() {
		RETURN_STRING_RULE(qi::raw[qi::double_]);
	}

	StringRule &xsd_value() {
		RETURN_STRING_RULE(atom() | number());
	}
}
