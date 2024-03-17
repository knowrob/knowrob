/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_FORMULA_PARSERS_H
#define KNOWROB_FORMULA_PARSERS_H

#include <boost/spirit/include/qi.hpp>
#include "knowrob/formulas/Formula.h"
#include "knowrob/formulas/Predicate.h"

namespace knowrob::parsers::formula {
	using FormulaRule = boost::spirit::qi::rule<std::string::const_iterator, std::shared_ptr<Formula>(), boost::spirit::ascii::space_type>;
	using PredicateRule = boost::spirit::qi::rule<std::string::const_iterator, std::shared_ptr<Predicate>(), boost::spirit::ascii::space_type>;

	/**
	 * @return a parser for formulas.
	 */
	FormulaRule &formula();

	/**
	 * @return a parser for predicates.
	 */
	PredicateRule &predicate();
}

#endif //KNOWROB_FORMULA_PARSERS_H
