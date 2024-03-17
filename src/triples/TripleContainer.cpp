/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/triples/TripleContainer.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

ProxyTripleContainer::ProxyTripleContainer(const std::vector<FramedTriplePtr> *triples)
		: triples_(triples) {
}

TripleContainer::ConstGenerator ProxyTripleContainer::cgenerator() const {
	return [it = triples_->begin(), end = triples_->end()]()
			mutable -> const FramedTriplePtr * {
		if (it == end) {
			return nullptr;
		}
		auto &ptr = *it;
		++it;
		return &ptr;
	};
}

TripleViewBatch::TripleViewBatch(uint32_t batchSize)
		: batchSize_(batchSize), actualSize_(0), data_(batchSize) {
}

void TripleViewBatch::add(const FramedTriplePtr &triple) {
	if (actualSize_ < batchSize_) {
		auto &entry = data_[actualSize_++];
		if (entry.owned && entry.ptr) {
			delete entry.ptr;
		}
		if (triple.owned) {
			entry.ptr = triple.ptr;
			entry.owned = true;
			triple.owned = false;
		} else {
			entry.ptr = new FramedTripleCopy(*triple.ptr);
			entry.owned = true;
		}
	}
}

TripleContainer::ConstGenerator TripleViewBatch::cgenerator() const {
	return [this, i = 0]() mutable -> const FramedTriplePtr * {
		if (i < actualSize_) return &data_[i++];
		return nullptr;
	};
}

MutableTripleContainer::MutableGenerator TripleViewBatch::generator() {
	return [this, i = 0]() mutable -> FramedTriplePtr * {
		if (i < actualSize_) return &data_[i++];
		return nullptr;
	};
}

namespace knowrob::py {
	template<>
	void createType<TripleContainer>() {
		using namespace boost::python;
		class_<TripleContainer, std::shared_ptr<TripleContainer>, boost::noncopyable>
				("TripleContainer", no_init)
				.def("__iter__",
					 boost::python::iterator<TripleContainer, boost::python::return_value_policy<boost::python::copy_const_reference>>{});
	}
}
