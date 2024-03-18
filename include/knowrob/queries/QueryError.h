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
#include "knowrob/KnowRobError.h"

namespace knowrob {
	/**
	 * A querying-related runtime error.
	 */
	class QueryError : public KnowRobError {
	public:
		/**
		 * @tparam Args fmt-printable arguments.
		 * @param fmt A fmt string pattern.
		 * @param args list of arguments used to instantiate the pattern.
		 */
		template<typename ... Args>
		explicit QueryError(const char *fmt, Args &&... args)
				: KnowRobError("QueryError", fmt::format(fmt, args...)) {}
	};
}

#endif //KNOWROB_QUERY_ERROR_H_
