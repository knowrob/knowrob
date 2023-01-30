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
		 * Can only be called once as streams are invalidated after one evaluation.
		 */
		void start();

	protected:
		class Node;
		using NodePtr = std::shared_ptr<Node>;
		using NodeList = std::list<NodePtr>;
		class Stream;

		std::shared_ptr<ReasonerManager> reasonerManager_;
		std::shared_ptr<ManagedReasoner> builtinEvaluator_;
		std::shared_ptr<QueryResultQueue> outputQueue_;
		std::shared_ptr<QueryResultBroadcaster> outBroadcaster_;
		std::shared_ptr<QueryResultBroadcaster> inputStream_;
		std::shared_ptr<QueryResultStream::Channel> inputChannel_;
		std::list<std::shared_ptr<Blackboard::Stream>> reasonerInputs_;
		std::shared_ptr<const Query> goal_;
		NodeList initialNodes_;

		/** Stop all reasoning processes attached to segments. */
		void stop();

		void combineAdjacentNodes();

		void createPipeline(const std::shared_ptr<QueryResultBroadcaster> &inputStream,
							const std::shared_ptr<QueryResultBroadcaster> &outputStream);

		void print(const NodePtr &node={});

		static void addSuccessor(const NodePtr &predecessor, const NodePtr &successor);
		static void removeSuccessor(const NodePtr &predecessor, const NodePtr &successor);

		NodeList decomposeFormula(const std::shared_ptr<Formula> &phi, const NodeList &predecessors);
		NodeList decomposePredicate(const std::shared_ptr<PredicateFormula> &phi, const NodeList &predecessors);

		void createConjunctiveQueries(const std::shared_ptr<Node> &node);
		void createDisjunctiveQueries(NodeList &node);

		void createPipeline(const std::shared_ptr<QueryResultBroadcaster> &pipelineInput,
							const std::shared_ptr<QueryResultBroadcaster> &pipelineOutput,
							const std::shared_ptr<Node> &n0);

		void createReasonerPipeline(const std::shared_ptr<ManagedReasoner> &r,
									const std::shared_ptr<Query> &subQuery,
									const std::shared_ptr<QueryResultBroadcaster> &pipelineInput,
									const std::shared_ptr<QueryResultBroadcaster> &nodeOutput);

		std::shared_ptr<ManagedReasoner> selectBuiltInReasoner(
				const FormulaPtr &phi,
				const std::set<std::shared_ptr<ManagedReasoner>> &setOfReasoner);

		template<class T> static
		std::shared_ptr<T> createConnectiveFormula(const FormulaPtr &phi1, const FormulaPtr &phi2, FormulaType type);

		class Stream : public QueryResultStream {
		public:
			Stream(
					const std::shared_ptr<ManagedReasoner> &reasoner,
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
			std::shared_ptr<ManagedReasoner> reasoner_;
			std::shared_ptr<const Query> goal_;
			std::shared_ptr<QueryResultStream::Channel> outputStream_;
			uint32_t queryID_;
			std::atomic<bool> isQueryOpened_;
			std::atomic<bool> hasStopRequest_;
			void push(const QueryResultPtr &msg) override;
			friend class Blackboard;
		};

		class Node {
		public:
			Node(const std::shared_ptr<Formula> &phi,
				 const std::set<std::shared_ptr<ManagedReasoner>> &reasoner,
				 PredicateType predicateType);
			Node(const std::shared_ptr<Formula> &phi,
				 const std::shared_ptr<ManagedReasoner> &reasoner,
				 PredicateType predicateType);
		protected:
			std::shared_ptr<Formula> phi_;
			PredicateType predicateType_;
			std::set<std::shared_ptr<ManagedReasoner>> reasoner_;
			std::list<std::shared_ptr<Node>> successors_;
			std::list<std::shared_ptr<Node>> predecessors_;
			friend class Blackboard;
		};
	};
}

#endif //KNOWROB_BLACKBOARD_H_
