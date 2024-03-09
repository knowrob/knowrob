/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/triples/GraphUnion.h"

using namespace knowrob;

void GraphUnion::write(std::ostream &os) const {
	os << "Union(";
	for (int i = 0; i < terms_.size(); i++) {
		if (i > 0) {
			os << ", ";
		}
		os << *terms_[i];
	}
	os << ")";
}
