/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_FRAMED_BINDINGS_H
#define KNOWROB_FRAMED_BINDINGS_H

#include "knowrob/terms/Substitution.h"
#include "knowrob/triples/GraphSelector.h"

namespace knowrob {

	class FramedBindings : public Substitution {
	public:
		void setFrame(const std::shared_ptr<GraphSelector> &frame) { frame_ = frame; }

		auto &frame() const { return frame_; }

	protected:
		std::shared_ptr<GraphSelector> frame_;
	};

	using FramedBindingsPtr = std::shared_ptr<FramedBindings>;
	using FramedBindingsHandler = std::function<void(const FramedBindingsPtr &)>;

} // knowrob

#endif //KNOWROB_FRAMED_BINDINGS_H
