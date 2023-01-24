/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_BLACKBOARD_H_
#define KNOWROB_BLACKBOARD_H_

// STD
#include <list>
#include <memory>
// KnowRob
#include <knowrob/queries.h>
#include <knowrob/reasoner.h>

namespace knowrob {
	/**
	 * A board that manages the evaluation of a query.
	 */
	class Blackboard {
	public:
		Blackboard(
			const std::shared_ptr<ReasonerManager> &reasonerManager,
			const std::shared_ptr<QueryResultQueue> &outputQueue,
			const std::shared_ptr<const Query> &goal);
		
		~Blackboard();

		/**
		 * Cannot be copy-assigned.
		 */
		Blackboard(const Blackboard&) = delete;
		
		/**
		 * Starts the query evaluation.
		 * Can only be called once as streams are invalidated after
		 * one evaluation.
		 */
		void start();
		
		/**
		 * The input stream of a sub-query.
		 */
		class Stream : public QueryResultStream {
		public:
			Stream(
				const std::shared_ptr<ManagedReasoner> &reasoner,
				const std::shared_ptr<QueryResultStream::Channel> &outputStream,
				const std::shared_ptr<Query> &goal);
			~Stream();
			
			/**
			 * Stop the stream by sending EOS message.
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
			std::shared_ptr<ManagedReasoner> reasoner_;
			std::shared_ptr<const Query> goal_;
			std::shared_ptr<QueryResultStream::Channel> outputStream_;
			uint32_t queryID_;
			std::atomic<bool> isQueryOpened_;
			std::atomic<bool> hasStopRequest_;
			
			void push(const QueryResultPtr &msg) override;
			
			friend class Blackboard;
		};
		
		/**
		 * A segment on the blackboard dedicated to a sub-query.
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
		std::shared_ptr<const Query> goal_;
		
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

#endif //KNOWROB_BLACKBOARD_H_
