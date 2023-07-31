//
// Created by daniel on 30.07.23.
//

#ifndef KNOWROB_REDUNDANT_ANSWER_FILTER_H
#define KNOWROB_REDUNDANT_ANSWER_FILTER_H

#include "set"
#include "AnswerBroadcaster.h"

namespace knowrob {

    class RedundantAnswerFilter : public AnswerBroadcaster {
    public:
        RedundantAnswerFilter() = default;

    protected:
        std::set<std::size_t> previousAnswers_;

        // Override QueryResultStream
        void push(const AnswerPtr &msg) override;
    };

} // knowrob

#endif //KNOWROB_REDUNDANT_ANSWER_FILTER_H
