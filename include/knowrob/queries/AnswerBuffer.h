//
// Created by daniel on 31.03.23.
//

#ifndef KNOWROB_BUFFERED_ANSWER_STREAM_H
#define KNOWROB_BUFFERED_ANSWER_STREAM_H

#include "AnswerBroadcaster.h"
#include "AnswerQueue.h"

namespace knowrob {

    class AnswerBuffer : public AnswerBroadcaster {
    public:
        AnswerBuffer();

        void stopBuffering();

        std::shared_ptr<AnswerQueue> createQueue();

    protected:
        std::atomic<bool> isBuffering_;
        std::list<AnswerPtr> buffer_;

        // Override QueryResultStream
        void push(const AnswerPtr &msg) override;
    };

    using AnswerBufferPtr = std::shared_ptr<AnswerBuffer>;

} // knowrob

#endif //KNOWROB_ANSWER_BUFFERED_STREAM_H
