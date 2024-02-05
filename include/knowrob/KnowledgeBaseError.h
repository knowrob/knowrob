/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_KNOWLEDGE_BASE_ERROR_H_
#define KNOWROB_KNOWLEDGE_BASE_ERROR_H_

#include <string>
#include <fmt/core.h>

namespace knowrob {
	/**
	 * A knowledge base-related runtime error.
	 */
	class KnowledgeBaseError : public std::runtime_error {
	public:
		/**
		 * @tparam Args fmt-printable arguments.
		 * @param fmt A fmt string pattern.
		 * @param args list of arguments used to instantiate the pattern.
		 */
		template<typename ... Args> explicit KnowledgeBaseError(const char *fmt, Args&& ... args)
		: std::runtime_error(fmt::format(fmt, args...))
		{}
	};
}

#endif //KNOWROB_QUERY_ERROR_H_
