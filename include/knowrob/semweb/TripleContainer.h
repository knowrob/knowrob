/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_TRIPLE_CONTAINER_H
#define KNOWROB_TRIPLE_CONTAINER_H

#include <vector>
#include "knowrob/semweb/StatementData.h"

namespace knowrob::semweb {
	class TripleContainer {
	public:
		auto begin() const { return asVector().begin(); }

		auto end() const { return asVector().end(); }

		auto empty() const { return begin() == end(); }

		virtual const std::vector<StatementData> &asVector() const = 0;
	};

	using TripleContainerPtr = std::shared_ptr<TripleContainer>;
	using TripleHandler = std::function<void(semweb::TripleContainerPtr)>;
	using TripleFilter = std::function<bool(const StatementData&)>;
}

#endif //KNOWROB_TRIPLE_CONTAINER_H
