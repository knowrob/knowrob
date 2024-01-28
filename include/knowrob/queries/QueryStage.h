//
// Created by daniel on 15.07.23.
//

#ifndef KNOWROB_QUERY_STAGE_H
#define KNOWROB_QUERY_STAGE_H

#include "Answer.h"
#include "AnswerQueue.h"
#include "AnswerBuffer.h"
#include "AnswerBroadcaster.h"
#include "DependencyGraph.h"
#include "Query.h"
#include "knowrob/semweb/RDFLiteral.h"

namespace knowrob {
    /**
     * A step within a query pipeline.
     */
    class QueryStage : public AnswerBroadcaster {
    public:
		QueryStage(QueryContextPtr ctx);

        ~QueryStage();

        /**
         * Request the stage to stop any active processes.
         * This will not necessary cause the processes to immediately exit,
         * but will ensure no more messages will be pushed into the output
         * stream of this stage.
         */
        virtual void close() override;

        /**
         * @return true if no EOS has been received.
         */
        bool isQueryOpened() const { return isQueryOpened_; }

        /**
         * @return true if it has been requested that the stage stops any active processes.
         */
        bool hasStopRequest() const { return hasStopRequest_; }

    protected:
        std::atomic<bool> isQueryOpened_;
        std::atomic<bool> isAwaitingInput_;
        std::atomic<bool> hasStopRequest_;
        std::weak_ptr<QueryStage> selfWeakRef_;

        using ActiveQuery = std::pair<AnswerBufferPtr, std::shared_ptr<AnswerStream>>;
        std::list<ActiveQuery> activeQueries_;
		QueryContextPtr ctx_;

        void push(const AnswerPtr &msg) override;

		virtual AnswerBufferPtr pushSubstitution(const Substitution &substitution) = 0;

        void pushTransformed(const AnswerPtr &transformedAnswer,
                             std::list<ActiveQuery>::iterator graphQueryIterator);

        friend class QueryStageTransformer;
        friend class KnowledgeBase; // weak ref hack
	};

    using QueryPipelineStagePtr = std::shared_ptr<QueryStage>;

    class LiteralQueryStage : public QueryStage {
	public:
		LiteralQueryStage(RDFLiteralPtr literal, const QueryContextPtr &ctx);

    protected:
        const RDFLiteralPtr literal_;

        virtual AnswerBufferPtr submitQuery(const RDFLiteralPtr &literal) = 0;
		AnswerBufferPtr pushSubstitution(const Substitution &substitution) override;
	};

    class FormulaQueryStage : public QueryStage {
	public:
		FormulaQueryStage(FormulaPtr formula, const QueryContextPtr &ctx);

    protected:
        const FormulaPtr formula_;

        virtual AnswerBufferPtr submitQuery(const FormulaPtr &formula) = 0;
		AnswerBufferPtr pushSubstitution(const Substitution &substitution) override;
	};
} // knowrob

#endif //KNOWROB_QUERY_STAGE_H
