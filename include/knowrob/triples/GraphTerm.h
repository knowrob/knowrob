/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_GRAPH_TERM_H
#define KNOWROB_GRAPH_TERM_H

#include <iostream>

namespace knowrob {
	enum class GraphTermType {
		Sequence,
		Union,
		Pattern,
		Builtin
	};

	class GraphTerm {
	public:
		bool isPattern() const { return termType_ == GraphTermType::Pattern; }

		auto termType() const { return termType_; }

		virtual void write(std::ostream &os) const = 0;

	protected:
		GraphTermType termType_;

		explicit GraphTerm(GraphTermType termType) : termType_(termType) {}

		friend class GraphQuery;
	};

} // knowrob

namespace std {
	std::ostream &operator<<(std::ostream &os, const knowrob::GraphTerm &t);
}

#endif //v
