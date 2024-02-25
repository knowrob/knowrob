/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PYTHON_ERROR_H
#define KNOWROB_PYTHON_ERROR_H

#include "memory"
#include "knowrob/KnowRobError.h"

namespace knowrob {
	/**
	 * An error that is thrown when a Python error occurs.
	 * It contains the current error message, the type of the exception and the stack trace extracted
	 * from the Python interpreter.
	 */
	class PythonError : public KnowRobError {
	public:
		PythonError();
	protected:
		// error data parsed from Python
		struct ErrorData {
			std::string exc_type;
			std::string exc_msg;
			std::string exc_trace;
			std::optional<std::string> exc_file;
			std::optional<int> exc_line;
		};
		std::unique_ptr<ErrorData> errorData_;

		explicit PythonError(ErrorData *errorData);

		static ErrorData* makeErrorData();

		static ErrorData* emptyErrorData(ErrorData *errorData);
	};

} // knowrob

#endif //KNOWROB_PYTHON_ERROR_H
