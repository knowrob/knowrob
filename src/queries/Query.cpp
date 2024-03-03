//
// Created by daniel on 28.07.23.
//

#include "knowrob/queries/Query.h"

using namespace knowrob;

namespace knowrob {
	QueryContextPtr DefaultQueryContext() {
		return std::make_shared<QueryContext>();
	}
}

namespace std {
	std::ostream &operator<<(std::ostream &os, const knowrob::Query &q) { //NOLINT
		QueryWriter(q, os);
		return os;
	}
}
