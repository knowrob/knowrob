/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_GRAPH_SELECTOR_H
#define KNOWROB_GRAPH_SELECTOR_H

#include "optional"
#include "Perspective.h"

namespace knowrob {
	/**
	 * The data base can contain multiple graphs, and this selector
	 * is used to select a subset of them.
	 * For example, each point in time is conceptually a separate graph, and
	 * a query may only address a specific point in time, or time interval.
	 */
	struct GraphSelector {
		GraphSelector() : graph(nullptr),
						  uncertain(false),
						  occasional(false) {}

		/**
		 * The name of the graph, usually reflects the name of an ontology.
		 */
		const char *graph;
		/**
		 * The perspective of statement.
		 */
		std::optional<PerspectivePtr> perspective;
		/**
		 * True if occasional triples are considered.
		 */
		bool occasional;
		/**
		 * True if uncertain triples are considered.
		 */
		bool uncertain;
		/**
		 * The begin of the time interval of consideration.
		 */
		std::optional<double> begin;
		/**
		 * The end of the time interval of consideration.
		 */
		std::optional<double> end;
		/**
		 * The minimum confidence threshold for statements.
		 */
		std::optional<double> confidence;

		/**
		 * Compute the hash value of this selector.
		 * @return the hash value
		 */
		size_t hash() const;

		/**
		 * Merge this selector with another selector.
		 * @param other another selector.
		 * @return true if the merge was successful.
		 */
		bool mergeWith(const GraphSelector &other);

		/**
		 * Write this selector to a stream.
		 * @param os the stream
		 * @return the stream
		 */
		std::ostream &write(std::ostream &os) const;
	};

	// alias
	using GraphSelectorPtr = std::shared_ptr<const GraphSelector>;

	/**
	 * @return the default graph selector
	 */
	static GraphSelectorPtr DefaultGraphSelector() {
		static auto defaultSelector = std::make_shared<const GraphSelector>();
		return defaultSelector;
	}
}

namespace std {
	std::ostream &operator<<(std::ostream &os, const knowrob::GraphSelector &selector);
}

#endif //KNOWROB_GRAPH_SELECTOR_H
