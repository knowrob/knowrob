/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_GRAPH_UNION_H
#define KNOWROB_GRAPH_UNION_H

#include "knowrob/triples/GraphConnective.h"
#include "knowrob/triples/FramedTriplePattern.h"

namespace knowrob {
	/**
	 * A query connective where the terms represent different alternatives.
	 */
	class GraphUnion : public GraphConnective {
	public:
		GraphUnion() : GraphConnective(GraphTermType::Union) {}

		explicit GraphUnion(const std::vector<std::shared_ptr<GraphTerm>> &terms)
				: GraphConnective(GraphTermType::Sequence, terms) {}

		void write(std::ostream &os) const override;
	};
} // knowrob

#endif //KNOWROB_GRAPH_UNION_H
