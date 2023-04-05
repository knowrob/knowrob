//
// Created by daniel on 31.03.23.
//

#ifndef KNOWROB_BUFFERED_ANSWER_STREAM_H
#define KNOWROB_BUFFERED_ANSWER_STREAM_H

#include "AnswerBroadcaster.h"

namespace knowrob {

    class BufferedAnswerStream : public AnswerBroadcaster {

    };


    using BufferedAnswerStreamPtr = std::shared_ptr<BufferedAnswerStream>;

} // knowrob

#endif //KNOWROB_ANSWER_BUFFERED_STREAM_H
