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
#include "knowrob/reasoner/ReasonerManager.h"
#include <knowrob/ReasoningGraph.h>
#include <knowrob/queries/QueryResultQueue.h>
#include <knowrob/queries/QueryResultBroadcaster.h>

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
		 * Can only be called once as streams are invalidated after one evaluation.
		 */
		void start();

	protected:
		class Stream; // forward declaration

		std::shared_ptr<ReasonerManager> reasonerManager_;
		std::shared_ptr<DefinedReasoner> builtinEvaluator_;
		std::shared_ptr<QueryResultQueue> outputQueue_;
		std::shared_ptr<QueryResultBroadcaster> outBroadcaster_;
		std::shared_ptr<QueryResultBroadcaster> inputStream_;
		std::shared_ptr<QueryResultStream::Channel> inputChannel_;
		std::list<std::shared_ptr<Blackboard::Stream>> reasonerInputs_;
		std::shared_ptr<const Query> goal_;

		/** Stop all reasoning processes attached to segments. */
		void stop();

		ReasoningGraph decomposeFormula(const std::shared_ptr<Formula> &phi) const;

		ReasoningGraph decomposePredicate(const std::shared_ptr<PredicateFormula> &phi) const;

		void createReasoningPipeline(const std::shared_ptr<QueryResultBroadcaster> &pipelineInput,
									 const std::shared_ptr<QueryResultBroadcaster> &pipelineOutput);

		void createReasoningPipeline(const std::shared_ptr<QueryResultBroadcaster> &pipelineInput,
									 const std::shared_ptr<QueryResultBroadcaster> &pipelineOutput,
									 const std::shared_ptr<ReasoningGraph::Node> &n0);

		void createReasoningStep(const std::shared_ptr<DefinedReasoner> &managedReasoner,
								 const std::shared_ptr<Query> &subQuery,
								 const std::shared_ptr<QueryResultBroadcaster> &stepInput,
								 const std::shared_ptr<QueryResultBroadcaster> &stepOutput);

		class Stream : public QueryResultStream {
		public:
			Stream(
					const std::shared_ptr<DefinedReasoner> &reasoner,
					const std::shared_ptr<QueryResultStream::Channel> &outputStream,
					const std::shared_ptr<Query> &goal);
			~Stream();
			// Stop the stream by sending EOS message.
			void stop();
			// @return true if the stream is opened.
			bool isQueryOpened() const { return isQueryOpened_; }
			// @return true if stop has been requested.
			bool hasStopRequest() const { return hasStopRequest_; }
		protected:
			std::shared_ptr<DefinedReasoner> reasoner_;
			std::shared_ptr<const Query> goal_;
			std::shared_ptr<QueryResultStream::Channel> outputStream_;
			uint32_t queryID_;
			std::atomic<bool> isQueryOpened_;
			std::atomic<bool> hasStopRequest_;
			void push(const QueryResultPtr &msg) override;
			friend class Blackboard;
		};
	};
}

#endif //KNOWROB_BLACKBOARD_H_
