/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/KnowRobError.h>
#include <knowrob/Logger.h>

using namespace knowrob;

static inline std::string makeWhat(std::string_view exc_type, std::string_view exc_msg, std::string_view exc_trace) {
	if (exc_trace.empty()) {
		return fmt::format("{}: {}", exc_type, exc_msg);
	} else {
		return fmt::format("{}: {}\n{}", exc_type, exc_msg, exc_trace);
	}
}

KnowRobError::KnowRobError(std::string_view exc_type, std::string_view exc_msg)
		: std::runtime_error(makeWhat(exc_type,exc_msg,"")), line_(0) {
}

KnowRobError::KnowRobError(std::string_view exc_type, std::string_view exc_msg, std::string_view exc_trace)
		: std::runtime_error(makeWhat(exc_type,exc_msg,exc_trace)), line_(0) {
}

KnowRobError::KnowRobError(std::string_view exc_type, std::string_view exc_msg, std::string_view exc_file, int exc_line)
		: std::runtime_error(makeWhat(exc_type,exc_msg,"")), file_(exc_file), line_(exc_line) {
}

KnowRobError::KnowRobError(std::string_view exc_type, std::string_view exc_msg, std::string_view exc_file, int exc_line, std::string_view exc_trace)
		: std::runtime_error(makeWhat(exc_type,exc_msg,exc_trace)), file_(exc_file), line_(exc_line) {
}
