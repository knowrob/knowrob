/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_GRAPH_TRANSFORMATION_RULE_H
#define KNOWROB_GRAPH_TRANSFORMATION_RULE_H

#include "knowrob/queries/SPARQLQuery.h"

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
		GraphTransformationRule(const std::vector<RDFLiteralPtr> &from, const std::vector<RDFLiteralPtr> &to)
				: from_(from), to_(to) {}

		/**
		 * @return the SPARQL query that is used to match the input graph.
		 */
		SPARQLQuery getSPARQLQuery() const { return SPARQLQuery(from_); }

		/**
		 * @return the statements to match in the input graph.
		 */
		const std::vector<RDFLiteralPtr> &from() const { return from_; }

		/**
		 * @return the statements to replace in the input graph.
		 */
		const std::vector<RDFLiteralPtr> &to() const { return to_; }

	protected:
		std::vector<RDFLiteralPtr> from_;
		std::vector<RDFLiteralPtr> to_;
	};

} // knowrob

#endif //KNOWROB_GRAPH_TRANSFORMATION_RULE_H
