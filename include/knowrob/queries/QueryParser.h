/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERY_PARSER_H
#define KNOWROB_QUERY_PARSER_H

#include "knowrob/formulas/Formula.h"
#include "knowrob/formulas/Predicate.h"
#include "knowrob/terms/Function.h"

namespace knowrob {
	/**
	 * Constructs formulae from strings.
	 */
	class QueryParser {
	public:
		static FormulaPtr parse(const std::string &queryString);

		static PredicatePtr parsePredicate(const std::string &queryString);

		static FunctionPtr parseFunction(const std::string &queryString);

		static TermPtr parseConstant(const std::string &queryString);

		static std::string parseRawAtom(const std::string &queryString);
	};

} // knowrob

#endif //KNOWROB_QUERY_PARSER_H
