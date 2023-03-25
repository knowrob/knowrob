//
// Created by daniel on 21.03.23.
//

#ifndef KNOWROB_QUERY_PARSER_H
#define KNOWROB_QUERY_PARSER_H

#include "knowrob/formulas/Formula.h"
#include "knowrob/formulas/Predicate.h"

namespace knowrob {
	// note: forward declared to avoid including parser library in the header.
	//       struct is defined in c++ file.
	struct ParserRules;

    /**
     * Constructs formulae from strings.
     */
    class QueryParser {
    protected:
        QueryParser();
        ~QueryParser();

        static ParserRules* get();

	public:
		static FormulaPtr parse(const std::string &queryString);

        static PredicatePtr parsePredicate(const std::string &queryString);

        static TermPtr parseConstant(const std::string &queryString);

	protected:
		ParserRules *bnf_;
	};

} // knowrob

#endif //KNOWROB_QUERY_PARSER_H
