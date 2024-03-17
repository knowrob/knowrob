/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_IMPORT_HIERARCHY_H
#define KNOWROB_IMPORT_HIERARCHY_H

#include "string"
#include "set"
#include "map"
#include "CurrentGraph.h"

namespace knowrob {
	/**
	 * Manages a hierarchy between named triple origins.
	 * One of the purposes of this hierarchy is to distinguish between triples that are dynamically
	 * inserted during runtime from those that are more static like files loaded initially.
	 */
	class ImportHierarchy {
	public:
		/**
		 * Used to match triples of any origin in a query.
		 */
		static constexpr std::string_view ORIGIN_ANY = "any";
		/**
		 * Used to only match system triples.
		 */
		static constexpr std::string_view ORIGIN_SYSTEM = "system";
		/**
		 * Used to only match triples from the current session, i.e. those that are inserted
		 * by the user, by reasoners or unittests during the current session.
		 */
		static constexpr std::string_view ORIGIN_SESSION = "session";
		/**
		 * Used to only match triples from the current user.
		 */
		static constexpr std::string_view ORIGIN_USER = "user";
		/**
		 * Used to only match triples from reasoners.
		 */
		static constexpr std::string_view ORIGIN_REASONER = "reasoner";
		/**
		 * Used to only match triples from unittests.
		 */
		static constexpr std::string_view ORIGIN_TEST = "test";

		ImportHierarchy();

		/**
		 * @param graphName a graph name
		 * @return true if graphName is known in this hierarchy.
		 */
		bool isCurrentGraph(std::string_view graphName) const;

		/**
		 * @param origin a graph name
		 * @return true if origin is a reserved origin.
		 */
		static bool isReservedOrigin(std::string_view origin);

		/**
		 * Clear the hierarchy.
		 */
		void clear() { graphs_.clear(); }

		/**
		 * Set the default graph used for triples in case no graph name is specified.
		 * @param defaultGraph a graph name.
		 */
		void setDefaultGraph(std::string_view defaultGraph) { defaultGraph_ = defaultGraph; }

		/**
		 * @return the default graph name of this hierarchy.
		 */
		const auto &defaultGraph() const { return defaultGraph_; }

		/**
		 * Defines a named graph if it is not defined yet.
		 * @param graphName a graph name.
		 */
		void addCurrentGraph(std::string_view graphName);

		/**
		 * Removes any definition of a named graph.
		 * @param graphName a graph name.
		 */
		void removeCurrentGraph(std::string_view graphName);

		/**
		 * Adds the imports relation between two named graphs.
		 * @param importerGraphName a graph name.
		 * @param importedGraphName a graph name.
		 */
		void addDirectImport(std::string_view importerGraphName, std::string_view importedGraphName);

		/**
		 * @param graphName a graph name.
		 * @return the transitive closure of the import relation starting from graphName
		 */
		const std::set<CurrentGraph *> &getImports(std::string_view graphName);

	protected:
		std::map<std::string_view, std::unique_ptr<CurrentGraph>> graphs_;
		std::string defaultGraph_;

		CurrentGraph& getCurrentGraph(std::string_view name);

		bool isSystemOrigin(CurrentGraph &graph);

		bool isSessionOrigin(CurrentGraph &graph);
	};

} // knowrob

#endif //KNOWROB_IMPORT_HIERARCHY_H
