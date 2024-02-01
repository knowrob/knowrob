/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TOKEN_H
#define KNOWROB_TOKEN_H

#include <memory>
#include <map>
#include <vector>
#include <string>

namespace knowrob {
	/**
	 * The type of a token.
	 */
	enum class TokenType : uint8_t {
		// A control token is used to control the evaluation pipeline.
		CONTROL_TOKEN = 0,
		// An answer token is the result of a query evaluation.
		ANSWER_TOKEN
	};

	/**
	 * A token is a single element in a query evaluation pipeline.
	 */
	class Token {
	public:
		Token() = default;

		/**
		 * @return the type of this token.
		 */
		virtual TokenType type() const = 0;

		/**
		 * @return true if this token is a control token.
		 */
		bool isControlToken() const { return type() == TokenType::CONTROL_TOKEN; }

		/**
		 * @return true if this token is an answer token.
		 */
		bool isAnswerToken() const { return type() == TokenType::ANSWER_TOKEN; }

		/**
		 * @return the hash of this token.
		 */
		virtual size_t hash() const;

		/**
		 * @return true if this token indicates the end of an evaluation.
		 */
		virtual bool indicatesEndOfEvaluation() const = 0;

		/**
		 * Write this token to the given output stream.
		 * @param os the output stream.
		 * @return the output stream.
		 */
		virtual std::ostream &write(std::ostream &os) const = 0;

		/**
		 * @return a human readable string representation of this token.
		 */
		std::string toString() const;
	};

	// alias
	using TokenPtr = std::shared_ptr<const Token>;
	using TokenMap = std::map<uint32_t, std::vector<TokenPtr>>;
}

namespace std {
	std::ostream &operator<<(std::ostream &os, const knowrob::Token &tok);

	std::ostream &operator<<(std::ostream &os, const knowrob::TokenPtr &tok);
}

#endif //KNOWROB_TOKEN_H
