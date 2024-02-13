/*
 * Copyright (c) 2024, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_ERROR_H_
#define KNOWROB_ERROR_H_

#include <string>
#include <exception>
#include <fmt/core.h>

namespace knowrob {

	/**
	 * A knowrob runtime error with optional stacktrace.
	 */
	class BaseError : public std::runtime_error {
	private:
		std::string stacktrace; // Field to store the stacktrace

	public:
		/**
		 * Constructor for error without stacktrace.
		 * @tparam Args fmt-printable arguments.
		 * @param fmt A fmt string pattern.
		 * @param args list of arguments used to instantiate the pattern.
		 */
		template<typename ... Args>
		explicit BaseError(const char *fmt, Args&& ... args)
				: std::runtime_error(fmt::format(fmt, std::forward<Args>(args)...))
		{}

		/**
		 * Constructor for error with stacktrace.
		 * @tparam Args fmt-printable arguments.
		 * @param fmt A fmt string pattern.
		 * @param stacktrace String representing the captured stack trace.
		 * @param args list of arguments used to instantiate the pattern.
		 */
		template<typename ... Args>
		explicit BaseError(const char *fmt, const std::string& stacktrace, Args&& ... args)
				: std::runtime_error(fmt::format(fmt, std::forward<Args>(args)...)),
				  stacktrace(stacktrace) // Initialize the stacktrace with the argument passed
		{}

		/**
 		* New constructor for error with simple string message and optional stacktrace.
 		* @param msg The error message.
 		* @param stacktrace (Optional) The stack trace associated with the error.
 		*/
		explicit BaseError(const std::string& msg, const std::string& stacktrace)
				: std::runtime_error(msg), stacktrace(stacktrace)
		{}

		/**
		 * Overrides the what() function to include the stack trace with the error message.
		 * @return A C-string with the error message and stack trace.
		 */
		const char* what() const noexcept override {
			static std::string fullMessage; // To ensure the lifetime of the returned c_str()
			fullMessage = std::runtime_error::what(); // Original error message
			if (!stacktrace.empty()) {
				fullMessage += "\nStack Trace:\n" + stacktrace;
			}
			return fullMessage.c_str();
		}
	};

}

#endif // KNOWROB_ERROR_H_