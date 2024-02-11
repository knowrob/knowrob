/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REASONER_ERROR_H_
#define KNOWROB_REASONER_ERROR_H_

#include <fmt/core.h>
#include "knowrob/BaseError.h"

namespace knowrob {
	/**
	 * A reasoner-related runtime error.
	 */
	class ReasonerError : public BaseError {
	public:
		// Constructor forwarding to BaseError with fmt string and arguments
		template<typename ... Args>
		explicit ReasonerError(const char *fmt, Args&& ... args)
				: BaseError(fmt, std::forward<Args>(args)...) {}

		// Constructor for a simple message without stack trace
		explicit ReasonerError(const std::string &msg)
				: BaseError(msg.c_str()) {}

		// Constructor for error with simple string message and optional stacktrace
		explicit ReasonerError(const std::string& msg, const std::string& stacktrace)
				: BaseError(msg, stacktrace) {}

		// Constructor with fmt string, stack trace, and arguments
		template<typename ... Args>
		explicit ReasonerError(const char *fmt, const std::string& stacktrace, Args&& ... args)
				: BaseError(fmt, stacktrace, std::forward<Args>(args)...) {}
	};
}

#endif // KNOWROB_REASONER_ERROR_H_