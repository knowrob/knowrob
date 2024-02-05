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
#include "ReasoningGraph.h"
#include "knowrob/queries/TokenQueue.h"
#include "knowrob/queries/TokenBroadcaster.h"

namespace knowrob {
	/**
	 * A board that manages the evaluation of a query.
	 */
	class Blackboard {
	public:
		Blackboard(ReasonerManager *reasonerManager,
			const std::shared_ptr<AnswerQueue> &outputQueue,
			const std::shared_ptr<const ModalQuery> &goal);
		
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

		ReasonerManager *reasonerManager_;
		std::shared_ptr<DefinedReasoner> builtinEvaluator_;
		std::shared_ptr<AnswerQueue> outputQueue_;
		std::shared_ptr<AnswerBroadcaster> outBroadcaster_;
		std::shared_ptr<AnswerBroadcaster> inputStream_;
		std::shared_ptr<AnswerStream::Channel> inputChannel_;
		std::list<std::shared_ptr<Blackboard::Stream>> reasonerInputs_;
		std::shared_ptr<const ModalQuery> goal_;

		/** Stop all reasoning processes attached to segments. */
		void stop();

		ReasoningGraph decomposeFormula(const std::shared_ptr<Formula> &phi) const;

		ReasoningGraph decomposePredicate(const std::shared_ptr<Predicate> &phi) const;

		void createReasoningPipeline(const std::shared_ptr<AnswerBroadcaster> &pipelineInput,
									 const std::shared_ptr<AnswerBroadcaster> &pipelineOutput);

		void createReasoningPipeline(const std::shared_ptr<AnswerBroadcaster> &pipelineInput,
									 const std::shared_ptr<AnswerBroadcaster> &pipelineOutput,
									 const std::shared_ptr<ReasoningGraph::Node> &n0);

		void createReasoningStep(const std::shared_ptr<DefinedReasoner> &managedReasoner,
								 const std::shared_ptr<ModalQuery> &subQuery,
								 const std::shared_ptr<AnswerBroadcaster> &stepInput,
								 const std::shared_ptr<AnswerBroadcaster> &stepOutput);

		class Stream : public AnswerStream {
		public:
			Stream(
					const std::shared_ptr<DefinedReasoner> &reasoner,
					const std::shared_ptr<AnswerStream::Channel> &outputStream,
					const std::shared_ptr<ModalQuery> &goal);
			~Stream();
			// Stop the stream by sending EOS message.
			void stop();
			// @return true if the stream is opened.
			bool isQueryOpened() const { return isQueryOpened_; }
			// @return true if stop has been requested.
			bool hasStopRequest() const { return hasStopRequest_; }
		protected:
			std::shared_ptr<DefinedReasoner> reasoner_;
			std::shared_ptr<const ModalQuery> goal_;
			std::shared_ptr<AnswerStream::Channel> outputStream_;
			uint32_t queryID_;
			std::atomic<bool> isQueryOpened_;
			std::atomic<bool> hasStopRequest_;
			void push(const TokenPtr &msg) override;
			friend class Blackboard;
		};
	};
}

#endif //KNOWROB_BLACKBOARD_H_
