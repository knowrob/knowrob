//
// Created by daniel on 27.03.23.
//

#ifndef KNOWROB_QUERY_PIPELINE_H
#define KNOWROB_QUERY_PIPELINE_H

#include "Answer.h"
#include "AnswerQueue.h"
#include "BufferedAnswers.h"
#include "AnswerBroadcaster.h"
#include "DependencyGraph.h"
#include "QueryEngine.h"

namespace knowrob {
    /**
     * A step within a query pipeline.
     */
    class ModalPipelineStage : public AnswerBroadcaster {
    public:
        explicit ModalPipelineStage(const std::list<LiteralPtr> &literals,
                                    const ModalityLabelPtr &label={});

        ~ModalPipelineStage();

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
        std::list<BufferedAnswersPtr> graphQueries_;

        void push(const AnswerPtr &msg) override;

        static AnswerPtr transformAnswer(const AnswerPtr &graphQueryAnswer, const AnswerPtr &partialResult);

        void pushTransformed(const AnswerPtr &transformedAnswer,
                             std::list<BufferedAnswersPtr>::iterator graphQueryIterator);

        friend class QueryProcessor;
        friend class MultiModalPipeline;
    };

    using QueryPipelineStagePtr = std::shared_ptr<ModalPipelineStage>;
    // forward declaration of internal data structure
    class QueryPipelineNode;

    /**
     * A pipeline where stages generate partial results to an input query.
     */
    class MultiModalPipeline {
    public:
        explicit MultiModalPipeline(const BufferedAnswersPtr &outputStream={});

        void setQueryEngine(QueryEngine *queryEngine);

        void setQueryFlags(int flags);

        const auto& outputQueue() const { return outputStream_; }

        const auto& stages() const { return stages_; }

        auto numStages() const { return stages_.size(); }

        void addModalGroup(const std::list<DependencyNodePtr> &dependencyGroup);

        void run();

    protected:
        QueryEngine *queryEngine_;
        BufferedAnswersPtr outputStream_;
        std::shared_ptr<AnswerBroadcaster> outBroadcaster_;
        std::shared_ptr<AnswerBroadcaster> inputStream_;
        std::shared_ptr<AnswerStream::Channel> inputChannel_;
        std::list<QueryPipelineStagePtr> stages_;
        int queryFlags_;

        void generate(const std::shared_ptr<QueryPipelineNode> &node_,
                      const std::shared_ptr<AnswerBroadcaster> &qnInput,
                      const std::shared_ptr<AnswerBroadcaster> &pipelineOutput);
    };

} // knowrob

#endif //KNOWROB_QUERY_PIPELINE_H
