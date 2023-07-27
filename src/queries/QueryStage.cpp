//
// Created by daniel on 15.07.23.
//

#include "knowrob/Logger.h"
#include "knowrob/queries/QueryStage.h"
#include "knowrob/queries/AnswerTransformer.h"

using namespace knowrob;

QueryStage::QueryStage(const std::list<LiteralPtr> &literals,
                       const ModalityLabelPtr &label)
        : AnswerBroadcaster(),
          queryEngine_(nullptr),
          queryFlags_((int)QueryFlag::QUERY_FLAG_ALL_SOLUTIONS),
          literals_(literals),
          label_(label),
          isQueryOpened_(true),
          hasStopRequest_(false)
{
}

QueryStage::~QueryStage()
{
    stop();
}

void QueryStage::setQueryEngine(QueryEngine *queryEngine)
{
    queryEngine_ = queryEngine;
}

void QueryStage::setQueryFlags(int queryFlags)
{
    queryFlags_ = queryFlags;
}

void QueryStage::stop()
{
    // toggle on stop request
    hasStopRequest_ = true;
    // close all channels
    close();
    // clear all graph queries
    for(auto &graphQuery : graphQueries_) graphQuery->close();
    graphQueries_.clear();
    // make sure EOS is published on this stream
    pushToBroadcast(AnswerStream::eos());
}

void QueryStage::pushTransformed(const AnswerPtr &transformedAnswer,
                                 std::list<AnswerBufferPtr>::iterator graphQueryIterator)
{
    if(AnswerStream::isEOS(transformedAnswer)) {
        graphQueries_.erase(graphQueryIterator);
        // only push EOS message if no graph query is still active and
        // if the stream has received EOS as input already.
        if(graphQueries_.empty() && !isQueryOpened()) {
            pushToBroadcast(transformedAnswer);
        }
    }
    else if(isQueryOpened()) {
        pushToBroadcast(transformedAnswer);
        // close the stage if only one solution is requested
        if((queryFlags_ & (int)QueryFlag::QUERY_FLAG_ONE_SOLUTION) == (int)QueryFlag::QUERY_FLAG_ONE_SOLUTION) stop();
    }
}

AnswerPtr QueryStage::transformAnswer(const AnswerPtr &graphQueryAnswer,
                                      const AnswerPtr &partialResult)
{
    if(AnswerStream::isEOS(graphQueryAnswer) || partialResult->substitution()->empty()) {
        return graphQueryAnswer;
    }
    else {
        // TODO: maybe would be good to avoid copy here. actually in this case
        //       it should be ok to modify input graphQueryAnswer, but it is handed in as const
        //       here because in the general case modification would not be ok.
        auto combined = std::make_shared<Answer>(*graphQueryAnswer);
        // TODO: could be done without unification
        combined->combine(partialResult);
        return combined;
    }
}

void QueryStage::push(const AnswerPtr &partialResult)
{
    if(AnswerStream::isEOS(partialResult)) {
        if(isQueryOpened()) {
            isQueryOpened_ = false;
        }
        // only broadcast EOS if no graph query is still active.
        if(graphQueries_.empty() && !hasStopRequest_) {
            pushToBroadcast(partialResult);
        }
    }
    else if(!isQueryOpened()) {
        KB_WARN("ignoring attempt to write to a closed stream.");
    }
    else if(!queryEngine_) {
        KB_ERROR("no query engine has been assigned.");
    }
    else {
        // apply the substitution mapping
        std::vector<LiteralPtr> literalInstances(literals_.size());
        unsigned long nextInstance=0;
        if(partialResult->substitution()->empty()) {
            for(auto &literal : literals_) {
                literalInstances[nextInstance++] = literal;
            }
        }
        else {
            for(auto &literal : literals_) {
                literalInstances[nextInstance++] = literal->applySubstitution(*partialResult->substitution());
            }
        }

        // create a new graph query
        auto graphQueryStream = queryEngine_->submitQuery(std::make_shared<GraphQuery>(
                literalInstances, queryFlags_, ModalityFrame(label_->modalOperators())));
        // keep a reference on the stream
        graphQueries_.push_front(graphQueryStream);
        auto graphQueryIt = graphQueries_.begin();
        // combine graph query answer with partialResult and push it to the broadcast
        graphQueryStream >> std::make_shared<AnswerTransformer>(
                [this,partialResult,graphQueryIt](const AnswerPtr &graphQueryAnswer) {
                    auto transformed = transformAnswer(graphQueryAnswer, partialResult);
                    pushTransformed(transformed, graphQueryIt);
                });
        // start sending messages into AnswerTransformer.
        // the messages are buffered before to avoid them being lost before the transformer
        // is connected.
        graphQueryStream->stopBuffering();
    }
}
