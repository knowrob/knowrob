/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/triples/GraphSequence.h"

using namespace knowrob;

void GraphSequence::write(std::ostream &os) const {
	os << "(";
	for (int i = 0; i < terms_.size(); i++) {
		if (i > 0) {
			os << ", ";
		}
		os << *terms_[i];
	}
	os << ")";
}
