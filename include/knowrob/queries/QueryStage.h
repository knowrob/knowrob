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
#include "QueryEngine.h"

namespace knowrob {
    /**
     * A step within a query pipeline.
     */
    class QueryStage : public AnswerBroadcaster {
    public:
        explicit QueryStage(RDFLiteralPtr literal, int queryFlags=Query::defaultFlags());

        ~QueryStage();

        void setQueryFlags(int flags);

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
        const RDFLiteralPtr literal_;
        std::atomic<bool> isQueryOpened_;
        std::atomic<bool> isAwaitingInput_;
        std::atomic<bool> hasStopRequest_;
        std::weak_ptr<QueryStage> selfWeakRef_;

        using ActiveQuery = std::pair<AnswerBufferPtr, std::shared_ptr<AnswerStream>>;
        std::list<ActiveQuery> graphQueries_;
        int queryFlags_;

        void push(const AnswerPtr &msg) override;

        virtual AnswerBufferPtr submitQuery(const RDFLiteralPtr &literal) = 0;

        void pushTransformed(const AnswerPtr &transformedAnswer,
                             std::list<ActiveQuery>::iterator graphQueryIterator);

        friend class QueryStageTransformer;
        friend class KnowledgeBase; // weak ref hack
    };

    using QueryPipelineStagePtr = std::shared_ptr<QueryStage>;
} // knowrob

#endif //KNOWROB_QUERY_STAGE_H
