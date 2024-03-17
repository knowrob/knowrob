/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TERM_PARSERS_H
#define KNOWROB_TERM_PARSERS_H

#include <boost/spirit/include/qi.hpp>
#include "knowrob/terms/Term.h"
#include "knowrob/terms/Atom.h"
#include "knowrob/terms/Function.h"

namespace knowrob::parsers::terms {
	using TermRule = boost::spirit::qi::rule<std::string::const_iterator, std::shared_ptr<Term>(), boost::spirit::ascii::space_type>;
	using AtomRule = boost::spirit::qi::rule<std::string::const_iterator, std::shared_ptr<Atom>(), boost::spirit::ascii::space_type>;
	using FunctionRule = boost::spirit::qi::rule<std::string::const_iterator, std::shared_ptr<Function>(), boost::spirit::ascii::space_type>;

	AtomRule &iri();

	AtomRule &atom_regular();

	AtomRule &atom();

	TermRule &string();

	TermRule &number();

	TermRule &blank();

	TermRule &xsd();

	TermRule &atomic();

	TermRule &var_upper();

	TermRule &var_question();

	TermRule &var();

	TermRule &atomic_list();

	TermRule &nil();

	TermRule &key_value_pair();

	TermRule &option();

	TermRule &options();

	TermRule &options_or_nil();

	TermRule &term();

	FunctionRule &function();
}

#endif //KNOWROB_TERM_PARSERS_H
