//
// Created by daniel on 31.03.23.
//

#include "knowrob/queries/BufferedAnswers.h"
#include "knowrob/queries/AnswerQueue.h"

using namespace knowrob;

BufferedAnswers::BufferedAnswers()
: AnswerBroadcaster(), isBuffering_(true)
{}

void BufferedAnswers::stopBuffering()
{
    isBuffering_ = false;
    for(auto &buffered : buffer_) AnswerBroadcaster::push(buffered);
    buffer_.clear();
}

std::shared_ptr<AnswerQueue> BufferedAnswers::createQueue()
{
    auto queue = std::shared_ptr<AnswerQueue>();
    addSubscriber(Channel::create(queue));
    stopBuffering();
    return queue;
}

void BufferedAnswers::push(const AnswerPtr &msg)
{
    if(isBuffering_) buffer_.push_back(msg);
    else AnswerBroadcaster::push(msg);
}
