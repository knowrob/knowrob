/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERY_RESULT_COMBINER_H_
#define KNOWROB_QUERY_RESULT_COMBINER_H_

#include <mutex>
#include <knowrob/queries/AnswerBroadcaster.h>

namespace knowrob {
	/**
	 * Combines multiple query result streams, and broadcasts each
	 * combination computed.
	 * This is intended to be used for parallel evaluation of
	 * sub-goals within a query.
	 */
	class AnswerCombiner : public AnswerBroadcaster {
	public:
		AnswerCombiner();
	
	protected:
		AnswerBuffer buffer_;
		std::mutex buffer_mutex_;

		// Override QueryResultStream
		void push(const Channel &channel, const AnswerPtr &msg) override;
		
		void genCombinations(uint32_t pushedChannelID,
                             AnswerBuffer::iterator it,
                             std::shared_ptr<Answer> &combinedResult);
	};
}

#endif //KNOWROB_QUERY_RESULT_COMBINER_H_
