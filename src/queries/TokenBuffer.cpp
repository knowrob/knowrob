/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/queries/TokenBuffer.h"
#include "knowrob/queries/TokenQueue.h"
#include "knowrob/Logger.h"
#include "knowrob/py/utils.h"

using namespace knowrob;

TokenBuffer::TokenBuffer()
		: TokenBroadcaster(), isBuffering_(true) {}

void TokenBuffer::stopBuffering() {
	if (isBuffering_) {
		isBuffering_ = false;
		for (auto &buffered: buffer_) {
			TokenBroadcaster::push(buffered);
		}
		buffer_.clear();
	}
}

std::shared_ptr<TokenQueue> TokenBuffer::createQueue() {
	auto queue = std::make_shared<TokenQueue>();
	addSubscriber(Channel::create(queue));
	stopBuffering();
	// TODO: should keep reference on buffer?
	return queue;
}

void TokenBuffer::push(const TokenPtr &tok) {
	if (isBuffering_) {
		buffer_.push_back(tok);
	} else {
		TokenBroadcaster::push(tok);
	}
}

namespace knowrob::py {
	template<>
	void createType<TokenBuffer>() {
		using namespace boost::python;
		class_<TokenBuffer, std::shared_ptr<TokenBuffer>, bases<TokenBroadcaster>, boost::noncopyable>
				("TokenBuffer", init<>())
				.def("stopBuffering", &TokenBuffer::stopBuffering)
				.def("createQueue", &TokenBuffer::createQueue);
	}
}
