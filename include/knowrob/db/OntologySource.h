/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_ONTOLOGY_SOURCE_H
#define KNOWROB_ONTOLOGY_SOURCE_H

#include "knowrob/triples/GraphSelector.h"

namespace knowrob {

	class OntologySource {
	public:
		void setFrame(const GraphSelectorPtr &frame) { frame_ = frame; }

		const auto &frame() const { return frame_; }

		/**
		 * The meaning is that parent origin imports this ontology file.
		 * @param parentOrigin the origin of the parent ontology.
		 */
		void setParentOrigin(std::string_view parentOrigin) { parentOrigin_ = parentOrigin; }

		/**
		 * @return the origin of the parent ontology.
		 */
		auto &parentOrigin() const { return parentOrigin_; }

		/**
		 * @return the origin identifier of the ontology.
		 */
		virtual std::string_view origin() const = 0;

	protected:
		GraphSelectorPtr frame_;
		std::optional<std::string> parentOrigin_;
	};

} // knowrob

#endif //KNOWROB_ONTOLOGY_SOURCE_H
