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
		auto begin() const { return asImmutableVector().begin(); }

		auto end() const { return asImmutableVector().end(); }

		auto empty() const { return asImmutableVector().begin() == asImmutableVector().end(); }

		virtual const std::vector<StatementData> &asImmutableVector() const = 0;
	};

	class ProxyTripleContainer : public TripleContainer {
	public:
		explicit ProxyTripleContainer(const std::vector<StatementData> *triples) : triples_(triples) {}

		const std::vector<StatementData>& asImmutableVector() const override { return *triples_; }
	protected:
		const std::vector<StatementData> *triples_;
	};

	class MutableTripleContainer : public TripleContainer {
	public:
		// TODO: is there a better way to do this? there are some context where a container can only
		//   be initialized with const data, so any non const methods are not available.
		auto beginMutable() { return asMutableVector().begin(); }

		auto endMutable() { return asMutableVector().end(); }

		virtual std::vector<StatementData> &asMutableVector() = 0;
	};

	using ImmutableTripleContainer = TripleContainer;

	using TripleContainerPtr = std::shared_ptr<TripleContainer>;
	using TripleHandler = std::function<void(const semweb::TripleContainerPtr&)>;

	using MutableTripleContainerPtr = std::shared_ptr<MutableTripleContainer>;
	using MutableTripleHandler = std::function<void(const semweb::MutableTripleContainerPtr&)>;

	using TripleFilter = std::function<bool(const StatementData&)>;
}

#endif //KNOWROB_TRIPLE_CONTAINER_H
