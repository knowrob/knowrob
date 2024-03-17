/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <boost/spirit/include/qi.hpp>
#include "knowrob/queries/QueryParser.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/knowrob.h"
#include "knowrob/queries/parsers/strings.h"
#include "knowrob/queries/parsers/terms.h"
#include "knowrob/queries/parsers/formula.h"

using namespace knowrob;

template<typename ResultType, typename RuleType>
static inline ResultType parse_(const std::string &queryString, const RuleType &rule) {
	auto first = queryString.begin();
	auto last = queryString.end();

	ResultType result;
	bool r = boost::spirit::qi::phrase_parse(first, last, rule, boost::spirit::ascii::space, result);

	if (first == last && r) {
		return result;
	} else {
		throw QueryError("Query string ({}) has invalid syntax.", queryString);
	}
}

FormulaPtr QueryParser::parse(const std::string &queryString) {
	return parse_<FormulaPtr, knowrob::parsers::formula::FormulaRule>(queryString, knowrob::parsers::formula::formula());
}

PredicatePtr QueryParser::parsePredicate(const std::string &queryString) {
	return parse_<PredicatePtr, knowrob::parsers::formula::PredicateRule>(queryString, knowrob::parsers::formula::predicate());
}

FunctionPtr QueryParser::parseFunction(const std::string &queryString) {
	return parse_<FunctionPtr, knowrob::parsers::terms::FunctionRule>(queryString, knowrob::parsers::terms::function());
}

TermPtr QueryParser::parseConstant(const std::string &queryString) {
	return parse_<TermPtr, knowrob::parsers::terms::TermRule>(queryString, knowrob::parsers::terms::atomic());
}

std::string QueryParser::parseRawAtom(const std::string &queryString) {
	return parse_<std::string, knowrob::parsers::str::StringRule>(queryString, knowrob::parsers::str::atom_or_iri());
}
