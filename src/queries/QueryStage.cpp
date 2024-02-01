/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <utility>

#include "knowrob/Logger.h"
#include "knowrob/KnowledgeBase.h"
#include "knowrob/queries/QueryStage.h"
#include "knowrob/queries/AnswerTransformer.h"
#include "knowrob/queries/AnswerYes.h"
#include "knowrob/queries/AnswerMerger.h"

using namespace knowrob;

QueryStage::QueryStage(QueryContextPtr ctx)
		: TokenBroadcaster(),
		  ctx_(std::move(ctx)),
		  isQueryOpened_(true),
		  isAwaitingInput_(true),
		  hasStopRequest_(false) {
}

QueryStage::~QueryStage() {
	QueryStage::close();
}

void QueryStage::close() {
	if (hasStopRequest_) return;

	// toggle on stop request
	hasStopRequest_ = true;

	// clear all graph queries
	for (auto &pair: activeQueries_) {
		pair.first->close();
		pair.second->close();
	}
	activeQueries_.clear();

	// close all channels
	TokenStream::close();
}

void QueryStage::pushTransformed(const TokenPtr &transformedTok,
								 std::list<ActiveQuery>::iterator graphQueryIterator) {
	if (transformedTok->indicatesEndOfEvaluation()) {
		activeQueries_.erase(graphQueryIterator);
		// only push EOS message if no query is still active and
		// if the stream has received EOS as input already.
		if (activeQueries_.empty() && !isAwaitingInput_) {
			isQueryOpened_ = false;
			pushToBroadcast(transformedTok);
		}
	} else if (isQueryOpened()) {
		// FIXME: pushing of "no" must be deferred, because it could be that another
		// instance of the query is asked with response "yes" later.
		// TODO: also set the ungrounded literals based on the more general query here for negative answers.
		//xxx;
		pushToBroadcast(transformedTok);
		// close the stage if only one solution is requested
		if ((ctx_->queryFlags_ & (int) QueryFlag::QUERY_FLAG_ONE_SOLUTION) ==
			(int) QueryFlag::QUERY_FLAG_ONE_SOLUTION) {
			pushToBroadcast(EndOfEvaluation::get());
			close();
		}
	}
}

void QueryStage::push(const TokenPtr &tok) {
	if (tok->indicatesEndOfEvaluation()) {
		// EOS indicates that no more input is to be expected
		isAwaitingInput_ = false;

		// only broadcast EOS if no graph query is still active.
		if (activeQueries_.empty() && !hasStopRequest_) {
			isQueryOpened_ = false;
			pushToBroadcast(tok);
		}
	} else if (tok->isAnswerToken()) {
		auto answer = std::static_pointer_cast<const Answer>(tok);

		if (!isQueryOpened()) {
			KB_WARN("ignoring attempt to write to a closed stream.");
		} else if (answer->isPositive()) {
			auto positiveAnswer = std::static_pointer_cast<const AnswerYes>(answer);

			// create a reference on self from a weak reference
			auto selfRef = selfWeakRef_.lock();
			if (!selfRef) return;

			// submit a query
			auto graphQueryStream = submitQuery(*positiveAnswer->substitution());

			// combine graph query answer with partialResult.
			// note that this is done in the same thread as the graph query is executed.
			auto merger = std::make_shared<AnswerMerger>(positiveAnswer);
			graphQueryStream >> merger;

			// from there it must be pushed into this stage again.
			// the stage could be destroyed in the meantime. but through the weak reference counter we can
			// create a step here that holds a reference to this stage.
			auto pusher = std::make_shared<Pusher>(selfRef);
			auto pair = activeQueries_.emplace_front(graphQueryStream, pusher);
			auto graphQueryIt = activeQueries_.begin();
			pusher->graphQueryIterator_ = graphQueryIt;
			merger >> pusher;

			// start sending messages into AnswerTransformer.
			// the messages are buffered before to avoid them being lost before the transformer
			// is connected.
			graphQueryStream->stopBuffering();
		} else {
			// the previous stage has already determined that the query is not satisfiable,
			// or had no evidence to conclude it is true or false.
			// we can stop here, and avoid evaluation of remaining literals.
			pushToBroadcast(answer);
		}
	}
}

QueryStage::Pusher::Pusher(std::shared_ptr<QueryStage> stage)
		: TokenStream(), stage_(std::move(stage)) {
}

void QueryStage::Pusher::close() {
	{
		std::lock_guard<std::mutex> lock(pushLock_);
		stage_ = {};
	}
	TokenStream::close();
}

void QueryStage::Pusher::push(const TokenPtr &tok) {
	if (stage_) {
		std::lock_guard<std::mutex> lock(pushLock_);
		stage_->pushTransformed(tok, graphQueryIterator_);
	}
}
