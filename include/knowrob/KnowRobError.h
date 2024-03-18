/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_KNOWROB_ERROR_H
#define KNOWROB_KNOWROB_ERROR_H

#include <fmt/core.h>
#include <optional>

namespace knowrob {
	/**
	 * A runtime error that is thrown when an error occurs in KnowRob.
	 */
	class KnowRobError : public std::runtime_error {
	public:
		/**
		 * @param exc_type A short description of the exception type.
		 * @param msg A message describing the error.
		 */
		KnowRobError(std::string_view exc_type, std::string_view msg);

		/**
		 * @param exc_type A short description of the exception type.
		 * @param msg A message describing the error.
		 * @param trace A stack trace.
		 */
		KnowRobError(std::string_view exc_type, std::string_view msg, std::string_view trace);

		/**
		 * @param exc_type A short description of the exception type.
		 * @param msg A message describing the error.
		 * @param file The file where the error occurred.
		 * @param line The line where the error occurred.
		 */
		KnowRobError(std::string_view exc_type, std::string_view msg, std::string_view file, int line);

		/**
		 * @param exc_type A short description of the exception type.
		 * @param msg A message describing the error.
		 * @param file The file where the error occurred.
		 * @param line The line where the error occurred.
		 * @param trace A stack trace.
		 */
		KnowRobError(std::string_view exc_type, std::string_view msg, std::string_view file, int line, std::string_view trace);

		/**
		 * @param file The file where the error occurred.
		 */
		void setFile(std::string_view file) { file_ = file; }

		/**
		 * @param line The line where the error occurred.
		 */
		void setLine(int line) { line_ = line; }

		/**
		 * @return the file where the error occurred.
		 */
		auto file() const { return file_; }

		/**
		 * @return the line where the error occurred.
		 */
		auto line() const { return line_; }

		/**
		 * @return true if the error has a file, false otherwise.
		 */
		bool hasFile() const { return file_.has_value(); }

	protected:
		std::optional<std::string> file_;
		int line_;
	};
}

#define LOG_KNOWROB_ERROR(err, msg, ...) \
    if(err.hasFile()) { \
        KB_ERROR("{}. Origin of error is:", fmt::format(msg, ##__VA_ARGS__)); \
        KB_ERROR1(err.file()->c_str(), err.line(), err.what()); \
    } else { \
        KB_ERROR("{}: {}", fmt::format(msg, ##__VA_ARGS__), err.what()); \
	}

#endif //KNOWROB_KNOWROB_ERROR_H
