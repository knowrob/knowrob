//
// Created by daniel on 30.01.24.
//

#ifndef KNOWROB_END_OF_EVALUATION_H
#define KNOWROB_END_OF_EVALUATION_H

#include "memory"
#include "iostream"
#include "Token.h"

namespace knowrob {
	class EndOfEvaluation : public Token {
	private:
		EndOfEvaluation() = default;

	public:
		/**s
		 * @return the singleton instance.
		 */
		static auto &get() {
			static std::shared_ptr<const EndOfEvaluation> instance(new EndOfEvaluation());
			return instance;
		}

		// override Token
		bool indicatesEndOfEvaluation() const override { return true; }

		// override Token
		TokenType type() const override { return TokenType::CONTROL_TOKEN; }

		// override Token
		std::ostream &write(std::ostream &os) const override { return os << "EndOfEvaluation"; }

		// override Token
		size_t hash() const override { return std::hash<std::string>()("EndOfEvaluation"); }
	};
}

#endif //KNOWROB_END_OF_EVALUATION_H
