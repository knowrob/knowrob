/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_BLACKBOARD_H__
#define __KNOWROB_BLACKBOARD_H__

// STD
#include <list>
#include <memory>
// KnowRob
#include <knowrob/qa/queries.h>
#include <knowrob/reasoning/ReasonerManager.h>

namespace knowrob {
	/**
	 * A board that manages the evaluation of a query.
	 */
	class Blackboard {
	public:
		Blackboard(
			const std::shared_ptr<ReasonerManager> &reasonerManager,
			const std::shared_ptr<QueryResultQueue> &outputQueue,
			const std::shared_ptr<Query> &goal);
		
		// copy constructor is not supported for blackboards
		Blackboard(const Blackboard&) = delete;
		
		~Blackboard();
		
		/** Starts the query evaluation.
		 * Can only be called once as streams are invalidated after
		 * one evaluation.
		 */
		void start();
		
		/** The input stream of a sub-query.
		 */
		class Stream : public QueryResultStream {
		public:
			Stream(
				const std::shared_ptr<IReasoner> &reasoner,
				const std::shared_ptr<QueryResultStream::Channel> &outputStream,
				const std::shared_ptr<Query> &goal);
			~Stream();
			
			/** Stop the stream by sending EOS message.
			 */
			void stop();
			
			/**
			 * @return true if the stream is opened.
			 */
			bool isQueryOpened() const { return isQueryOpened_; }
			
			/**
			 * @return true if stop has been requested.
			 */
			bool hasStopRequest() const { return hasStopRequest_; }
		
		protected:
			std::shared_ptr<IReasoner> reasoner_;
			uint32_t queryID_;
			std::atomic<bool> isQueryOpened_;
			std::atomic<bool> hasStopRequest_;
			
			void push(const QueryResultPtr &msg);
			
			friend class Blackboard;
		};
		
		/** A segment on the blackboard dedicated to a sub-query.
		 */
		class Segment {
		public:
			Segment(const std::shared_ptr<Blackboard::Stream> &in,
				const std::shared_ptr<QueryResultBroadcaster> &out);
		
		protected:
			std::shared_ptr<Blackboard::Stream> in_;
			std::shared_ptr<QueryResultBroadcaster> out_;
			
			void stop();
			
			friend class Blackboard;
		};

	protected:
		std::shared_ptr<ReasonerManager> reasonerManager_;
		std::shared_ptr<QueryResultQueue> outputQueue_;
		std::shared_ptr<QueryResultBroadcaster> outBroadcaster_;
		std::shared_ptr<QueryResultBroadcaster> inputStream_;
		std::shared_ptr<QueryResultStream::Channel> inputChannel_;
		std::shared_ptr<Query> goal_;
		
		std::list<std::shared_ptr<Segment>> segments_;

		/** Decompose the blackboard into different segments. */
		void decompose(
			const std::shared_ptr<Formula> &phi,
			std::shared_ptr<QueryResultBroadcaster> &in,
			std::shared_ptr<QueryResultBroadcaster> &out);
		
		void decomposePredicate(
			const std::shared_ptr<PredicateFormula> &phi,
			std::shared_ptr<QueryResultBroadcaster> &in,
			std::shared_ptr<QueryResultBroadcaster> &out);
		
		void decomposeConjunction(
			const std::shared_ptr<ConjunctionFormula> &phi,
			std::shared_ptr<QueryResultBroadcaster> &in,
			std::shared_ptr<QueryResultBroadcaster> &out);
		
		void decomposeDisjunction(
			const std::shared_ptr<DisjunctionFormula> &phi,
			std::shared_ptr<QueryResultBroadcaster> &in,
			std::shared_ptr<QueryResultBroadcaster> &out);

		/** Stop all reasoning processes attached to segments. */
		void stop();
	};
}

#endif //__KNOWROB_BLACKBOARD_H__
