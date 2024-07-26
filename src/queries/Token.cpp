/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "sstream"
#include "knowrob/queries/Token.h"
#include "knowrob/knowrob.h"
#include "knowrob/integration/python/utils.h"
#include "knowrob/queries/EndOfEvaluation.h"
#include "knowrob/queries/Answer.h"

using namespace knowrob;

size_t Token::hash() const {
	size_t val = 0;
	switch (tokenType()) {
		case TokenType::CONTROL_TOKEN:
			// note currently there is only one control token type, so nothing needs to be done here for the moment.
			hashCombine(val, uint8_t(tokenType()));
			break;
		case TokenType::ANSWER_TOKEN:
			hashCombine(val, ((const Answer *) this)->hashOfAnswer());
			break;
	}
	return val;
}

std::string Token::stringForm() const {
	switch (tokenType()) {
		case TokenType::CONTROL_TOKEN:
			// note currently there is only one control token type, so nothing needs to be done here for the moment.
			return "EndOfEvaluation";
		case TokenType::ANSWER_TOKEN:
			return ((const Answer *) this)->stringFormOfAnswer();
	}
	return "UnknownToken";
}

std::ostream &std::operator<<(std::ostream &os, const knowrob::Token &tok) {
	return os << tok.stringForm();
}

std::ostream &std::operator<<(std::ostream &os, const knowrob::TokenPtr &tok) {
	return os << tok->stringForm();
}

namespace knowrob::py {
	template<>
	void createType<Token>() {
		using namespace boost::python;
		enum_<TokenType>("TokenType")
				.value("CONTROL_TOKEN", TokenType::CONTROL_TOKEN)
				.value("ANSWER_TOKEN", TokenType::ANSWER_TOKEN)
				.export_values();
		class_<Token, std::shared_ptr<Token>, boost::noncopyable>
				("Token", no_init)
				.def("__repr__", &Token::stringForm)
				.def("__hash__", &Token::hash)
				.def("tokenType", &Token::tokenType)
				.def("indicatesEndOfEvaluation", &Token::indicatesEndOfEvaluation);
		class_<EndOfEvaluation, bases<Token>, std::shared_ptr<EndOfEvaluation>>
				("EndOfEvaluation", no_init)
				.def("get", &EndOfEvaluation::get, return_value_policy<reference_existing_object>())
				.staticmethod("get");
		// allow implicit conversion from shared_ptr<Token> to shared_ptr<const Token>
		register_ptr_to_python< std::shared_ptr< const Token > >();
		implicitly_convertible< std::shared_ptr< Token >, std::shared_ptr< const Token > >();
		// create subtypes
		createType<Answer>();
	}
}
