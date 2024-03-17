/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_BACKEND_ERROR_H_
#define KNOWROB_BACKEND_ERROR_H_

#include <fmt/core.h>

namespace knowrob {
	/**
	 * A data backend-related runtime error.
	 */
	class BackendError : public std::runtime_error {
	public:
		/**
		 * @tparam Args fmt-printable arguments.
		 * @param fmt A fmt string pattern.
		 * @param args list of arguments used to instantiate the pattern.
		 */
		template<typename ... Args>
		explicit BackendError(const char *fmt, Args&& ... args)
		: std::runtime_error(fmt::format(fmt, args...)) {}
	};
}

#endif //KNOWROB_BACKEND_ERROR_H_
