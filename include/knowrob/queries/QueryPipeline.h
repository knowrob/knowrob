//
// Created by daniel on 27.03.23.
//

#ifndef KNOWROB_QUERY_PIPELINE_H
#define KNOWROB_QUERY_PIPELINE_H

#include "Answer.h"
#include "AnswerQueue.h"
#include "BufferedAnswerStream.h"
#include "AnswerBroadcaster.h"
#include "knowrob/KnowledgeBase.h"
#include "DependencyGraph.h"

namespace knowrob {
    /**
     * A step within a query pipeline.
     */
    class QueryPipelineStage : public AnswerBroadcaster {
    public:
        QueryPipelineStage(const std::list<LiteralPtr> &literals,
                           const ModalityLabelPtr &label={});

        ~QueryPipelineStage();

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

        void push(const AnswerPtr &msg) override;
        std::shared_ptr<BufferedAnswerStream> submitQuery();
        void handleAnswer(const AnswerPtr &answer);

        friend class QueryProcessor;
    };

    using QueryPipelineStagePtr = std::shared_ptr<QueryPipelineStage>;
    // forward declaration of internal data structure
    class QueryPipelineNode;

    /**
     * A pipeline where stages generate partial results to an input query.
     */
    class QueryPipeline {
    public:
        explicit QueryPipeline(const std::shared_ptr<AnswerQueue> &outputQueue={});

        const std::shared_ptr<AnswerQueue>& outputQueue() const { return outputQueue_; }

        const std::list<QueryPipelineStagePtr>& stages() const { return stages_; }

        auto numStages() const { return stages_.size(); }

        void add(const std::list<DependencyNodePtr> &dependencyGroup);

        void run();

    protected:
        std::shared_ptr<AnswerQueue> outputQueue_;
        std::shared_ptr<AnswerBroadcaster> outBroadcaster_;
        std::shared_ptr<AnswerBroadcaster> inputStream_;
        std::shared_ptr<AnswerStream::Channel> inputChannel_;
        std::list<QueryPipelineStagePtr> stages_;

        void generate(const std::shared_ptr<QueryPipelineNode> &node_,
                      const std::shared_ptr<AnswerBroadcaster> &qnInput,
                      const std::shared_ptr<AnswerBroadcaster> &pipelineOutput);
    };

} // knowrob

#endif //KNOWROB_QUERY_PIPELINE_H
