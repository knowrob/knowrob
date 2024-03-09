/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/triples/GraphTerm.h"

using namespace knowrob;

namespace std {
	std::ostream &operator<<(std::ostream &os, const knowrob::GraphTerm &t) {
		t.write(os);
		return os;
	}
}
