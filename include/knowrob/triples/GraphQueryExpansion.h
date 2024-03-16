/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_GRAPH_QUERY_EXPANSION_H
#define KNOWROB_GRAPH_QUERY_EXPANSION_H

#include "knowrob/queries/QueryContext.h"
#include "knowrob/triples/GraphPathQuery.h"
#include "knowrob/triples/GraphQuery.h"

namespace knowrob {
	struct GraphQueryExpansion {
		GraphQueryExpansion() : counter(0), with_reassignment(false) {}

		GraphPathQueryPtr original;
		GraphQueryPtr expanded;
		std::vector<VariablePtr> o_vars;
		std::vector<VariablePtr> u_vars;
		VariablePtr accumulated_begin;
		VariablePtr accumulated_end;
		QueryContextPtr query_ctx;
		uint32_t counter;
		bool with_reassignment;
	};

	using GraphQueryExpansionPtr = std::shared_ptr<GraphQueryExpansion>;
}

#endif //KNOWROB_GRAPH_QUERY_EXPANSION_H
