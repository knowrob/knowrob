/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_STRING_PARSERS_H
#define KNOWROB_STRING_PARSERS_H

#include <boost/spirit/include/qi.hpp>

namespace knowrob::parsers::str {
	using StringRule = boost::spirit::qi::rule<std::string::const_iterator, std::string()>;

	StringRule &single_quotes();

	StringRule &double_quotes();

	StringRule &lower_prefix();

	StringRule &upper_prefix();

	// a IRI namespace is an alphanumeric word.
	// note that no single quotes are allowed, the greedy parser would match the `singleQuotes` rule before.
	StringRule &iri_ns();

	// right part of colon must be an alphanumeric word, or wrapped in single quoted.
	// Note that there is no need to enquote entities whose name starts with an uppercase character.
	StringRule &iri_entity();

	// IRIs are encoded as "ns:entity", ns must be a registered namespace at parse-time
	StringRule &iri();

	// Atoms are either a single quoted string, or words that start with a lowercase letter.
	StringRule &atom();

	// A typed IRI or another type atom
	StringRule &atom_or_iri();

	// A single blank character
	StringRule &blank();

	// A number as a string
	StringRule &number();

	// The value of a typed XSD literal as a string
	StringRule &xsd_value();
}

#endif //KNOWROB_STRING_PARSERS_H
