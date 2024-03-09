/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_GRAPH_TRANSFORMATION_RULE_H
#define KNOWROB_GRAPH_TRANSFORMATION_RULE_H

#include "knowrob/triples/SPARQLQuery.h"

namespace knowrob {
	/**
	 * A graph transformation rule that restructures and possibly renames entities in the input graph.
	 */
	class GraphTransformationRule {
	public:
		/**
		 * @from the statements to match in the input graph.
		 * @to the statements to replace in the input graph.
		 */
		GraphTransformationRule(const std::vector<FramedTriplePatternPtr> &from, const std::vector<FramedTriplePatternPtr> &to)
				: from_(from), to_(to) {}

		/**
		 * @return the SPARQL query that is used to match the input graph.
		 */
		SPARQLQuery getSPARQLQuery() const { return SPARQLQuery(std::make_shared<GraphQuery>(from_)); }

		/**
		 * @return the statements to match in the input graph.
		 */
		const std::vector<FramedTriplePatternPtr> &from() const { return from_; }

		/**
		 * @return the statements to replace in the input graph.
		 */
		const std::vector<FramedTriplePatternPtr> &to() const { return to_; }

	protected:
		std::vector<FramedTriplePatternPtr> from_;
		std::vector<FramedTriplePatternPtr> to_;
	};

} // knowrob

#endif //KNOWROB_GRAPH_TRANSFORMATION_RULE_H
