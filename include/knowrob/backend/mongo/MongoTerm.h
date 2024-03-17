/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGO_TERM_H
#define KNOWROB_MONGO_TERM_H

#include <mongoc.h>
#include "knowrob/terms/Term.h"

namespace knowrob::mongo {
	/**
	 * Translates a Term to a MongoDB query.
	 */
	class MongoTerm {
	public:
		static void append(
				bson_t *doc,
				const char *key,
				const TermPtr &term,
				const char *queryOperator = nullptr,
				bool matchNullValue = false,
				bool includeVariables = false);

		static void appendWithVars(
				bson_t *doc,
				const char *key,
				const TermPtr &term,
				const char *queryOperator = nullptr,
				bool matchNullValue = false);

		static void append(
				bson_t *doc,
				const char *key,
				const std::vector<TermPtr> &terms,
				const char *arrayOperator = "$or");

		static std::string variableKey(const std::string_view &varName);
	};
} // knowrob

#endif //KNOWROB_MONGO_TERM_H
