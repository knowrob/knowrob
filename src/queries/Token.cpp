/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "sstream"
#include "knowrob/queries/Token.h"
#include "knowrob/knowrob.h"

using namespace knowrob;

size_t Token::hash() const {
	size_t val = 0;
	hashCombine(val, uint8_t(type()));
	return val;
}

std::string Token::toString() const {
	std::stringstream ss;
	write(ss);
	return ss.str();
}

std::ostream &std::operator<<(std::ostream &os, const knowrob::Token &tok) {
	return tok.write(os);
}

std::ostream &std::operator<<(std::ostream &os, const knowrob::TokenPtr &tok) {
	return tok->write(os);
}
