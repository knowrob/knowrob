//
// Created by daniel on 21.03.23.
//

#ifndef KNOWROB_QUERY_PARSER_H
#define KNOWROB_QUERY_PARSER_H

#include "knowrob/formulas/Formula.h"

namespace knowrob {
	// note: forward declared to avoid including parser library in the header.
	//       struct is defined in c++ file.
	struct ParserRules;

    class QueryParser {
	public:
		QueryParser();

		QueryParser(const QueryParser&) = delete;

		~QueryParser();

		FormulaPtr parse(const std::string &queryString);

	protected:
		ParserRules *bnf_;
	};

} // knowrob

#endif //KNOWROB_QUERY_PARSER_H
