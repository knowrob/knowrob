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

    class QueryPipeline {
    public:
        QueryPipeline(const KnowledgeBasePtr &kb,
                      const std::shared_ptr<QueryResultQueue> &outputQueue);

        void addDependencyGroup(const std::list<ModalDependencyNode*> &dependencyGroup);

        void run();

    protected:
        KnowledgeBasePtr kb_;
        std::shared_ptr<QueryResultQueue> outputQueue_;
        std::shared_ptr<QueryResultBroadcaster> outBroadcaster_;
        std::shared_ptr<QueryResultBroadcaster> inputStream_;
        std::shared_ptr<QueryResultStream::Channel> inputChannel_;

        struct CompareNodes { bool operator()(
                    ModalDependencyNode *a,
                    ModalDependencyNode *b) const; };
        struct QueryNode {
            explicit QueryNode(const ModalDependencyNode *node);

            const ModalDependencyNode *node_;
            std::list<std::shared_ptr<QueryNode>> successors_;
            std::priority_queue<ModalDependencyNode*,
                std::vector<ModalDependencyNode*>, CompareNodes> neighbors_;
        };
        using QueryNodePtr = std::shared_ptr<QueryNode>;

        class Stream : public QueryResultStream {
        public:
            Stream(const KnowledgeBasePtr &kb,
                   const QueryNodePtr &qn,
                   const std::shared_ptr<QueryResultStream::Channel> &outChan);
            ~Stream();
            void stop();
            bool isQueryOpened() const { return isQueryOpened_; }
            bool hasStopRequest() const { return hasStopRequest_; }

        protected:
            const KnowledgeBasePtr kb_;
            const QueryNodePtr qn_;
            const std::shared_ptr<QueryResultStream::Channel> outChan_;
            std::atomic<bool> isQueryOpened_;
            std::atomic<bool> hasStopRequest_;
            void push(const QueryResultPtr &msg) override;
            friend class QueryProcessor;
        };

        std::list<std::shared_ptr<Stream>> queryStreams_;

        void createPipeline(std::shared_ptr<QueryNode> &qn,
                            const std::shared_ptr<QueryResultBroadcaster> &qnInput,
                            const std::shared_ptr<QueryResultBroadcaster> &pipelineOutput);
    };

} // knowrob

#endif //KNOWROB_QUERY_PIPELINE_H
