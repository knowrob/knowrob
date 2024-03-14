/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_GRAPH_RENAMING_H
#define KNOWROB_GRAPH_RENAMING_H

#include "string"
#include "map"
#include "GraphTransformation.h"
#include "knowrob/triples/FramedTriple.h"

namespace knowrob {
	/**
	 * A map of entity names to their renamed counterparts.
	 */
	using GraphRenamingMap = std::map<std::string, std::string, std::less<>>;

	/**
	 * A transformation that only renames entities in the input graph.
	 */
	class GraphRenaming : public GraphTransformation {
	public:
		GraphRenaming() = default;

		/**
		 * Create a new renaming transformation with the given renaming map.
		 * @param renaming the renaming map
		 */
		explicit GraphRenaming(GraphRenamingMap renaming);

		/**
		 * Apply renaming mapping to an entity.
		 * @param entity the entity to rename
		 * @return the renamed entity
		 */
		std::string_view rename(const std::string_view &entity);

		/**
		 * Rename the entities in the given triple.
		 * @param triple the triple to rename
		 */
		void rename(FramedTriple &triple);

		/**
		 * Add a renaming rule to the renaming map.
		 * @param from the original entity name
		 * @param to the new entity name
		 */
		void addRenaming(std::string_view from, std::string_view to);

		// override GraphTransformation
		bool configure(const boost::property_tree::ptree &opts) override;

		// override GraphTransformation
		void initializeTransformation() override;

		// override GraphTransformation
		void finalizeTransformation() override;

		// override GraphTransformation
		void pushInputTriples(const semweb::TripleContainerPtr &triples) override;

	protected:
		GraphRenamingMap renaming_;
	};

} // knowrob

#endif //KNOWROB_GRAPH_RENAMING_H
