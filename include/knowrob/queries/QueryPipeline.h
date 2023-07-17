//
// Created by daniel on 27.03.23.
//

#ifndef KNOWROB_QUERY_PIPELINE_H
#define KNOWROB_QUERY_PIPELINE_H

#include "Answer.h"
#include "AnswerQueue.h"
#include "AnswerBuffer.h"
#include "AnswerBroadcaster.h"
#include "DependencyGraph.h"
#include "QueryEngine.h"
#include "QueryStage.h"

namespace knowrob {
    // forward declaration of internal data structure
    class QueryPipelineNode;

    /**
     * A pipeline where stages generate partial results to an input query.
     */
    class QueryPipeline {
    public:
        explicit QueryPipeline(const std::shared_ptr<AnswerBroadcaster> &outputStream={});

        void setQueryEngine(QueryEngine *queryEngine);

        void setQueryFlags(int flags);

        const auto& outputStream() const { return outputStream_; }

        const auto& stages() const { return stages_; }

        auto numStages() const { return stages_.size(); }

        void addDependencyGroup(const std::list<DependencyNodePtr> &dependencyGroup);

        void run();

    protected:
        QueryEngine *queryEngine_;
        std::shared_ptr<AnswerBroadcaster> outputStream_;
        std::shared_ptr<AnswerBroadcaster> outputCombiner_;
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
