/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/queries/AnswerCombiner.h>

using namespace knowrob;

AnswerCombiner::AnswerCombiner()
: AnswerBroadcaster()
{}

void AnswerCombiner::push(const Channel &channel, const AnswerPtr &msg)
{
	const uint32_t channelID = channel.id();
	// need to lock the whole push as genCombinations uses an iterator over the buffer.
	std::lock_guard<std::mutex> lock(buffer_mutex_);
	
	// add to the buffer for later combinations
	buffer_[channelID].push_back(msg);
	
	// generate combinations with other channels if each channel
	// buffer has some content.
	if(buffer_.size() == channels_.size()) {
		std::shared_ptr<Answer> combination(new Answer(*(msg.get())));
		// generate all combinations and push combined messages
		genCombinations(channelID, buffer_.begin(), combination);
	}
}

void AnswerCombiner::genCombinations( //NOLINT
		uint32_t pushedChannelID,
        AnswerBuffer::iterator it,
        std::shared_ptr<Answer> &combinedResult)
{
	if(it == buffer_.end()) {
		// end reached, push combination
		// note: need to create a new SubstitutionPtr due to the rollBack below.
		// TODO: it could be that the same combination is generated multiple times, avoid redundant push here?
		AnswerBroadcaster::push(std::make_shared<Answer>(*combinedResult));
	}
	else if(it->first == pushedChannelID) {
		// ignore pushed channel
	}
	else if(it->second.size()==1) {
		// only a single message buffered from this channel
		auto it1 = it; ++it1;
		// keep track of changes for rolling them back
		Reversible changes;
		// combine and continue with next channel.
		// this is done to avoid creating many copies of the substitution map.
		if(combinedResult->combine(it->second.front(), &changes)) {
			genCombinations(pushedChannelID, it1, combinedResult);
		}
		// roll back changes
		changes.rollBack();
	}
	else {
		// generate a combination for each buffered message
		// note: the number of possible combinations grows
		// exponentially with number of messages in channels
		// TODO: it might be good to think about ways of discarding options quicker.
		// - one way would be to cache all possible combinations, but could be many of them.
		//   then only these would need to be iterated here, but additionally pre-computing
		//   must happen here
		auto it1 = it; ++it1;
		// keep track of changes for rolling them back
		Reversible changes;
		// combine each buffered message
		for(auto &msg : it->second) {
			// combine and continue with next channel
			if(combinedResult->combine(msg, &changes)) {
				genCombinations(pushedChannelID, it1, combinedResult);
			}
			// roll back changes, note this also clears the `changes` object
			// so it can be re-used in the next iteration
			changes.rollBack();
		}
	}
}
