/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_QUERY_STAGE_H
#define KNOWROB_QUERY_STAGE_H

#include "Query.h"
#include "Answer.h"
#include "TokenBuffer.h"
#include "TokenBroadcaster.h"
#include "AnswerNo.h"
#include "AnswerDontKnow.h"

namespace knowrob {
	/**
	 * A step within a query pipeline.
	 */
	class QueryStage : public TokenBroadcaster {
	public:
		explicit QueryStage(QueryContextPtr ctx);

		~QueryStage();

		/**
		 * Request the stage to stop any active processes.
		 * This will not necessary cause the processes to immediately exit,
		 * but will ensure no more messages will be pushed into the output
		 * stream of this stage.
		 */
		void close() override;

		/**
		 * @return true if query is still active.
		 */
		bool isQueryOpened() const { return isQueryOpened_; }

		/**
		 * @return true if it has been requested that the stage stops any active processes.
		 */
		bool hasStopRequest() const { return hasStopRequest_; }

	protected:
		/**
		 * Submits a query using given substitution mapping.
		 * @param substitution a mapping from variables to terms.
		 * @return an answer buffer.
		 */
		virtual TokenBufferPtr submitQuery(const Bindings &substitution) = 0;

	protected:
		std::atomic<bool> isQueryOpened_;
		std::atomic<bool> isAwaitingInput_;
		std::atomic<bool> hasStopRequest_;
		std::atomic<bool> hasPositiveAnswer_;
		std::weak_ptr<QueryStage> selfWeakRef_;
		std::vector<AnswerNoPtr> deferredNegativeAnswers_;
		std::vector<AnswerDontKnowPtr> deferredDontKnowAnswers_;

		using ActiveQuery = std::pair<TokenBufferPtr, std::shared_ptr<TokenStream>>;
		using ActiveQueryIter = std::list<ActiveQuery>::iterator;
		std::list<ActiveQuery> activeQueries_;
		QueryContextPtr ctx_;

		/**
		 * Pushes tokens into the output stream of a stage.
		 */
		class Pusher : public TokenStream {
		public:
			std::shared_ptr<QueryStage> stage_;
			ActiveQueryIter graphQueryIterator_;
			std::mutex pushLock_;

			explicit Pusher(std::shared_ptr<QueryStage> stage);

			void push(const TokenPtr &tok) override;

			void close() override;
		};

		// push a token into the output stream of this stage,
		// this is called by the Pusher objects.
		void pushTransformed(const TokenPtr &transformedToken, ActiveQueryIter graphQueryIterator);

		// override AnswerBroadcaster
		void push(const TokenPtr &tok) override;

		void pushDeferred();

		// needed for "weak ref hack"
		friend class KnowledgeBase;
	};

	/**
	 * A typed query stage that submits instances of a specific query type.
	 */
	template<class QueryType>
	class TypedQueryStage : public QueryStage {
	public:
		/**
		 * A lambda expression used to submit instances of the input query.
		 */
		using QuerySubmitter = std::function<TokenBufferPtr(const std::shared_ptr<QueryType> &)>;

		TypedQueryStage(const QueryContextPtr &ctx,
		                std::shared_ptr<QueryType> query,
		                const QuerySubmitter &submitter)
				: QueryStage(ctx),
				  query_(std::move(query)),
				  submitter_(submitter) {}

	protected:
		const std::shared_ptr<QueryType> query_;
		QuerySubmitter submitter_;

		// override QueryStage
		TokenBufferPtr submitQuery(const Bindings &substitution) override {
			// apply the substitution mapping
			auto instance = applyBindings(query_, substitution);
			// submit a query
			return submitter_(instance);
		}
	};
} // knowrob

#endif //KNOWROB_QUERY_STAGE_H
