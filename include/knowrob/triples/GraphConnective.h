/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_GRAPH_CONNECTIVE_H
#define KNOWROB_GRAPH_CONNECTIVE_H

#include "knowrob/triples/GraphTerm.h"
#include "knowrob/triples/GraphPattern.h"

namespace knowrob {
	/**
	 * A connective graph term with has as arguments a variadic number of other terms.
	 */
	class GraphConnective : public GraphTerm {
	public:
		/**
		 * Add a pattern to the graph connective.
		 * @param pattern the pattern to add.
		 */
		void addPattern(const FramedTriplePatternPtr &pattern) {
			if (pattern) terms_.push_back(std::make_shared<GraphPattern>(pattern));
		}

		/**
		 * Add a term to the graph connective.
		 * @param term the term to add.
		 */
		void addMember(const std::shared_ptr<GraphTerm> &member) {
			if (member) terms_.push_back(member);
		}

		/**
		 * @return the union of graph terms.
		 */
		const auto &terms() const { return terms_; }

	protected:
		std::vector<std::shared_ptr<GraphTerm>> terms_;

		explicit GraphConnective(GraphTermType termType) : GraphTerm(termType) {}

		explicit GraphConnective(GraphTermType termType, const std::vector<std::shared_ptr<GraphTerm>> &terms)
				: GraphTerm(termType), terms_(terms) {}
	};
} // knowrob

#endif //KNOWROB_GRAPH_CONNECTIVE_H
