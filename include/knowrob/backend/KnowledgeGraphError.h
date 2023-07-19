/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_BACKEND_ERROR_H_
#define KNOWROB_BACKEND_ERROR_H_

#include <fmt/core.h>

namespace knowrob {
	/**
	 * A data knowledgeGraph-related runtime error.
	 */
	class KnowledgeGraphError : public std::runtime_error {
	public:
		/**
		 * @tparam Args fmt-printable arguments.
		 * @param fmt A fmt string pattern.
		 * @param args list of arguments used to instantiate the pattern.
		 */
		template<typename ... Args>
		explicit KnowledgeGraphError(const char *fmt, Args&& ... args)
		: std::runtime_error(fmt::format(fmt, args...)) {}
	};
}

#endif //KNOWROB_REASONER_ERROR_H_
