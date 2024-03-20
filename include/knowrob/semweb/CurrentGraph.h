/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_CURRENT_GRAPH_H
#define KNOWROB_CURRENT_GRAPH_H

#include "string"
#include "set"
#include "map"

namespace knowrob {
	/**
	 * A named graph currently defined in a knowledge graph.
	 */
	class CurrentGraph {
	public:
		explicit CurrentGraph(const std::string_view &name)
				: name_(name) {}

		/**
		 * @param other another graph
		 * @return true if the graphs have the same name
		 */
		bool operator==(const CurrentGraph &other) const {
			return name_ == other.name_;
		}

		/**
		 * @return the name of the graph.
		 */
		const auto &name() const { return name_; }

		/**
		 * @return list of direct imports of this graph.
		 */
		const auto &directImports() const { return directImports_; }

		/**
		 * @return transitive closure of imports relation starting from this graph.
		 */
		const auto &imports() const { return imports_; }

	protected:
		const std::string name_;
		std::set<CurrentGraph *> directImports_;
		std::set<CurrentGraph *> imports_;

		friend class ImportHierarchy;
	};
} // knowrob

#endif //KNOWROB_CURRENT_GRAPH_H
