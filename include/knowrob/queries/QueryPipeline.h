//
// Created by daniel on 02.09.23.
//

#ifndef KNOWROB_QUERY_PIPELINE_H
#define KNOWROB_QUERY_PIPELINE_H

#include "memory"
#include "AnswerStream.h"

namespace knowrob {
    /**
     * Holds a reference to pipeline stages during execution,
     * and stops each stage on destruction ensuring that none of them
     * continues broadcasting messages.
     */
    class QueryPipeline {
    public:
        QueryPipeline() = default;
        ~QueryPipeline();

        void addStage(const std::shared_ptr<AnswerStream> &stage);

    protected:
        std::vector<std::shared_ptr<AnswerStream>> stages_;
    };
}


#endif //KNOWROB_QUERYPIPELINE_H
