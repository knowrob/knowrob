/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_ONTOLOGY_SOURCE_H
#define KNOWROB_ONTOLOGY_SOURCE_H

#include "knowrob/semweb/GraphSelector.h"

namespace knowrob {

	class OntologySource {
	public:
		void setFrame(const GraphSelectorPtr &frame) { frame_ = frame; }

		const auto &frame() const { return frame_; }

	protected:
		GraphSelectorPtr frame_;
	};

} // knowrob

#endif //KNOWROB_ONTOLOGY_SOURCE_H
