//
// Created by daniel on 15.07.23.
//

#include <utility>

#include "knowrob/Logger.h"
#include "knowrob/queries/QueryStage.h"
#include "knowrob/queries/AnswerTransformer.h"

using namespace knowrob;

QueryStage::QueryStage(RDFLiteralPtr literal, int queryFlags)
: AnswerBroadcaster(),
  literal_(std::move(literal)),
  queryFlags_(queryFlags),
  isQueryOpened_(true),
  isAwaitingInput_(true),
  hasStopRequest_(false)
{
    std::cout << "QueryStage::QueryStage " << this << std::endl;
}

QueryStage::~QueryStage()
{
    std::cout << "QueryStage::~QueryStage " << this << std::endl;
    stop();
}

void QueryStage::setQueryFlags(int queryFlags)
{
    queryFlags_ = queryFlags;
}

void QueryStage::stop()
{
    std::cout << "QueryStage::stop " << this << std::endl;
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
        // only push EOS message if no query is still active and
        // if the stream has received EOS as input already.
        if(graphQueries_.empty() && !isAwaitingInput_) {
            isQueryOpened_ = false;
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
        auto combined = std::make_shared<Answer>(*graphQueryAnswer);
        // TODO: could be done without unification
        combined->combine(partialResult);
        return combined;
    }
}

void QueryStage::push(const AnswerPtr &partialResult)
{
    if(AnswerStream::isEOS(partialResult)) {
        std::cout << "QueryStage::pushEOS " << this << std::endl;

        // EOS indicates that no more input is to be expected
        isAwaitingInput_ = false;

        // only broadcast EOS if no graph query is still active.
        if(graphQueries_.empty() && !hasStopRequest_) {
            isQueryOpened_ = false;
            pushToBroadcast(partialResult);
        }
    }
    else if(!isQueryOpened()) {
        KB_WARN("ignoring attempt to write to a closed stream.");
    }
    else {
        // apply the substitution mapping
        auto literalInstance =
            std::make_shared<RDFLiteral>(*literal_, *partialResult->substitution());

        // submit a query
        auto graphQueryStream = submitQuery(literalInstance);

        // keep a reference on the stream
        graphQueries_.push_front(graphQueryStream);
        auto graphQueryIt = graphQueries_.begin();

        // combine graph query answer with partialResult and push it to the broadcast
        graphQueryStream >> std::make_shared<AnswerTransformer>(
                [this,partialResult,graphQueryIt](const AnswerPtr &graphQueryAnswer) {
                    if(AnswerStream::isEOS(graphQueryAnswer)) {
                        pushTransformed(graphQueryAnswer, graphQueryIt);
                    }
                    else {
                        auto transformed = transformAnswer(graphQueryAnswer, partialResult);
                        pushTransformed(transformed, graphQueryIt);
                    }
                });

        // start sending messages into AnswerTransformer.
        // the messages are buffered before to avoid them being lost before the transformer
        // is connected.
        graphQueryStream->stopBuffering();
    }
}
