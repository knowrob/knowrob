/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REASONER_ERROR_H_
#define KNOWROB_REASONER_ERROR_H_

#include <fmt/core.h>

namespace knowrob {
	/**
	 * A reasoner-related runtime error.
	 */
	class ReasonerError : public std::runtime_error {
	public:
		/**
		 * @tparam Args fmt-printable arguments.
		 * @param fmt A fmt string pattern.
		 * @param args list of arguments used to instantiate the pattern.
		 */
		template<typename ... Args>
		explicit ReasonerError(const char *fmt, Args &&... args)
				: std::runtime_error(fmt::format(fmt, args...)) {}

		explicit ReasonerError(const std::string &msg)
				: std::runtime_error(msg) {}
	};
}

#endif //KNOWROB_REASONER_ERROR_H_
