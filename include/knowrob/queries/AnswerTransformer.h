//
// Created by daniel on 22.04.23.
//

#ifndef KNOWROB_ANSWER_TRANSFORMER_H
#define KNOWROB_ANSWER_TRANSFORMER_H

#include <knowrob/queries/Answer.h>
#include <knowrob/queries/AnswerStream.h>

#include <utility>

namespace knowrob {
    class AnswerTransformer : public AnswerStream {
    public:
        using TransformFunction = std::function<void(const AnswerPtr&)>;

        explicit AnswerTransformer(TransformFunction transformFunction)
        : AnswerStream(), transformFunction_(std::move(transformFunction)) {}

    protected:
        TransformFunction transformFunction_;

   		// Override QueryResultStream
   		void push(const AnswerPtr &msg) override { transformFunction_(msg); }
    };

} // knowrob

#endif //KNOWROB_ANSWER_TRANSFORMER_H
