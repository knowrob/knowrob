//
// Created by daniel on 30.07.23.
//

#include "knowrob/queries/RedundantAnswerFilter.h"

using namespace knowrob;

void RedundantAnswerFilter::push(const AnswerPtr &msg)
{
    auto msgHash = msg->computeHash();
    if(previousAnswers_.count(msgHash)==0) {
        AnswerBroadcaster::push(msg);
        previousAnswers_.insert(msgHash);
    }
}
