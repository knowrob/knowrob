/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/queries/Query.h"

using namespace knowrob;

namespace knowrob {
	QueryContextPtr DefaultQueryContext() {
		return std::make_shared<QueryContext>();
	}

	QueryContextPtr OneSolutionContext() {
		return std::make_shared<QueryContext>(QUERY_FLAG_ONE_SOLUTION);
	}
}

namespace std {
	std::ostream &operator<<(std::ostream &os, const knowrob::Query &q) { //NOLINT
		QueryWriter(q, os);
		return os;
	}
}
