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
        explicit QueryStage(const std::list<LiteralPtr> &literals,
                            const ModalityLabelPtr &label={});

        ~QueryStage();

        void setQueryEngine(QueryEngine *queryEngine);

        void setQueryFlags(int flags);

        /**
         * Request the stage to stop any active processes.
         * This will not necessary cause the processes to immediately exit,
         * but will ensure no more messages will be pushed into the output
         * stream of this stage.
         */
        void stop();

        /**
         * @return true if no EOS has been received.
         */
        bool isQueryOpened() const { return isQueryOpened_; }

        /**
         * @return true if it has been requested that the stage stops any active processes.
         */
        bool hasStopRequest() const { return hasStopRequest_; }

    protected:
        const std::list<LiteralPtr> literals_;
        const ModalityLabelPtr label_;
        std::atomic<bool> isQueryOpened_;
        std::atomic<bool> hasStopRequest_;

        QueryEngine *queryEngine_;
        int queryFlags_;
        std::list<AnswerBufferPtr> graphQueries_;

        void push(const AnswerPtr &msg) override;

        static AnswerPtr transformAnswer(const AnswerPtr &graphQueryAnswer, const AnswerPtr &partialResult);

        void pushTransformed(const AnswerPtr &transformedAnswer,
                             std::list<AnswerBufferPtr>::iterator graphQueryIterator);
    };

    using QueryPipelineStagePtr = std::shared_ptr<QueryStage>;
} // knowrob

#endif //KNOWROB_QUERY_STAGE_H
