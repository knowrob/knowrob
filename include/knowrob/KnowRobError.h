/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_KNOWROB_ERROR_H
#define KNOWROB_KNOWROB_ERROR_H

#include <fmt/core.h>
#include <optional>

namespace knowrob {
	class KnowRobError : public std::runtime_error {
	public:
		explicit KnowRobError(std::string_view exc_type, std::string_view msg);
		KnowRobError(std::string_view exc_type, std::string_view msg, std::string_view trace);
		KnowRobError(std::string_view exc_type, std::string_view msg, std::string_view file, int line);
		KnowRobError(std::string_view exc_type, std::string_view msg, std::string_view file, int line, std::string_view trace);

		void setFile(std::string_view file) { file_ = file; }
		void setLine(int line) { line_ = line; }

		auto file() const { return file_; }
		auto line() const { return line_; }

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
