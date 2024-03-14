/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_GRAPH_SEQUENCE_H
#define KNOWROB_GRAPH_SEQUENCE_H

#include "knowrob/triples/GraphConnective.h"

namespace knowrob {
	/**
	 * A query connective where all terms must be matched.
	 */
	class GraphSequence : public GraphConnective {
	public:
		GraphSequence() : GraphConnective(GraphTermType::Sequence) {}

		explicit GraphSequence(const std::vector<std::shared_ptr<GraphTerm>> &terms)
				: GraphConnective(GraphTermType::Sequence, terms) {}

		void write(std::ostream &os) const override;
	};
} // knowrob

#endif //KNOWROB_GRAPH_SEQUENCE_H
