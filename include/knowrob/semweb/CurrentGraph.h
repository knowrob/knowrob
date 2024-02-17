//
// Created by daniel on 09.04.23.
//

#ifndef KNOWROB_SEMWEB_CURRENT_GRAPH_H
#define KNOWROB_SEMWEB_CURRENT_GRAPH_H

#include "string"
#include "set"
#include "map"

namespace knowrob::semweb {
	/**
	 * A named graph currently defined in a knowledge graph.
	 */
	class CurrentGraph {
	public:
		explicit CurrentGraph(const std::string_view &name)
				: name_(name) {}

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
} // knowrob::semweb

#endif //KNOWROB_SEMWEB_CURRENT_GRAPH_H
