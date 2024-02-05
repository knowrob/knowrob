//
// Created by daniel on 30.07.23.
//

#include "knowrob/queries/RedundantAnswerFilter.h"

using namespace knowrob;

void RedundantAnswerFilter::push(const TokenPtr &tok) {
	auto msgHash = tok->hash();
	if (previousAnswers_.count(msgHash) == 0) {
		TokenBroadcaster::push(tok);
		previousAnswers_.insert(msgHash);
	}
}
