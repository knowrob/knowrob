//
// Created by daniel on 27.03.23.
//

#ifndef KNOWROB_QUERY_PIPELINE_H
#define KNOWROB_QUERY_PIPELINE_H

#include "QueryResult.h"
#include "QueryResultQueue.h"
#include "QueryResultBroadcaster.h"
#include "knowrob/KnowledgeBase.h"
#include "DependencyGraph.h"

namespace knowrob {
    /**
     * A step within a query pipeline.
     */
    class QueryPipelineStage : public QueryResultStream {
    public:
        QueryPipelineStage(const std::shared_ptr<QueryResultStream> &outStream,
                           const std::list<LiteralPtr> &literals,
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
        const std::shared_ptr<QueryResultStream::Channel> outChan_;
        std::atomic<bool> isQueryOpened_;
        std::atomic<bool> hasStopRequest_;

        void push(const QueryResultPtr &msg) override;
        std::shared_ptr<QueryResultBroadcaster> submitQuery();

        friend class QueryProcessor;
    };

    // forward declaration of internal data structure
    class QueryPipelineNode;

    /**
     * A pipeline where stages generate partial results to an input query.
     */
    class QueryPipeline {
    public:
        QueryPipeline(const std::shared_ptr<QueryResultQueue> &outputQueue);

        void addDependencyGroup(const std::list<DependencyNodePtr> &dependencyGroup);

        void run();

    protected:
        std::shared_ptr<QueryResultQueue> outputQueue_;
        std::shared_ptr<QueryResultBroadcaster> outBroadcaster_;
        std::shared_ptr<QueryResultBroadcaster> inputStream_;
        std::shared_ptr<QueryResultStream::Channel> inputChannel_;
        std::list<std::shared_ptr<QueryPipelineStage>> stages_;

        void generate(const std::shared_ptr<QueryPipelineNode> &node_,
                      const std::shared_ptr<QueryResultBroadcaster> &qnInput,
                      const std::shared_ptr<QueryResultBroadcaster> &pipelineOutput);
    };

} // knowrob

#endif //KNOWROB_QUERY_PIPELINE_H
