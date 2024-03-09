/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_GRAPH_PATTERN_H
#define KNOWROB_GRAPH_PATTERN_H

#include "knowrob/triples/GraphTerm.h"
#include "knowrob/triples/FramedTriplePattern.h"

namespace knowrob {
	/**
	 * A triple pattern that appears in a graph query.
	 */
	class GraphPattern : public GraphTerm {
	public:
		explicit GraphPattern(FramedTriplePatternPtr pattern)
				: pattern_(std::move(pattern)), GraphTerm(GraphTermType::Pattern) {}

		/**
		 * @return the triple pattern.
		 */
		const auto &value() const { return pattern_; }

		void write(std::ostream &os) const override { os << *pattern_; }

	protected:
		FramedTriplePatternPtr pattern_;
	};
} // knowrob

#endif //KNOWROB_GRAPH_PATTERN_H
