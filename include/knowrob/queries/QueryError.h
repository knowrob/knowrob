#ifndef KNOWROB_QUERY_ERROR_H_
#define KNOWROB_QUERY_ERROR_H_

#include "knowrob/BaseError.h" // Include the base class header
#include "knowrob/terms/Term.h" // For Term class
#include "knowrob/queries/FormulaQuery.h" // For Query class

namespace knowrob {
	/**
	 * A querying-related runtime error.
	 */
	class QueryError : public BaseError {
	private:
		std::string queryDescription; // Additional field to store query description

	public:
		// Constructor forwarding to BaseError with fmt string and arguments
		template<typename ... Args>
		explicit QueryError(const char *fmt, Args&& ... args)
				: BaseError(fmt, std::forward<Args>(args)...) {}

		// Constructor for a simple message without stack trace
		explicit QueryError(const std::string &msg)
				: BaseError(msg.c_str()) {}

		// Constructor for error with simple string message and optional stacktrace
		explicit QueryError(const std::string& msg, const std::string& stacktrace)
				: BaseError(msg, stacktrace) {}

		// Constructor with fmt string, stack trace, and arguments
		template<typename ... Args>
		explicit QueryError(const char *fmt, const std::string& stacktrace, Args&& ... args)
				: BaseError(fmt, stacktrace, std::forward<Args>(args)...) {}

		// Constructor with query and error term information
		QueryError(const Query &erroneousQuery, const Term &errorTerm)
				: BaseError("Query error"), // Initialize with a basic error message; adjust as needed
				  queryDescription(erroneousQuery.toString() + ": " + errorTerm.toString()) // Assuming these methods exist
		{}

		// Override what() to include query description if present
		const char* what() const noexcept override {
			static std::string fullMessage = BaseError::what(); // Start with base message
			if (!queryDescription.empty()) {
				fullMessage += "\nQuery Description: " + queryDescription;
			}
			return fullMessage.c_str();
		}
	};
}

#endif //KNOWROB_QUERY_ERROR_H_