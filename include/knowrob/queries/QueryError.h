/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERY_ERROR_H_
#define KNOWROB_QUERY_ERROR_H_

#include <string>
#include <fmt/core.h>
#include "knowrob/terms/Term.h"
#include <knowrob/queries/Query.h>

namespace knowrob {
	/**
	 * A querying-related runtime error.
	 */
	class QueryError : public std::runtime_error {
	public:
		/**
		 * @tparam Args fmt-printable arguments.
		 * @param fmt A fmt string pattern.
		 * @param args list of arguments used to instantiate the pattern.
		 */
		template<typename ... Args> explicit QueryError(const char *fmt, Args&& ... args)
		: std::runtime_error(fmt::format(fmt, args...))
		{}

		/**
		 * @param erroneousQuery the query that caused an error
		 * @param errorTerm a term denoting the error
		 */
		QueryError(const Query &erroneousQuery, const Term &errorTerm);
	
	protected:
		
		static std::string formatErrorString(const Query &erroneousQuery, const Term &errorTerm);
	};
}

#endif //KNOWROB_QUERY_ERROR_H_
