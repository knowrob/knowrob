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
#include "knowrob/queries/AnswerNo.h"

using namespace knowrob;

QueryStage::QueryStage(QueryContextPtr ctx)
		: TokenBroadcaster(),
		  ctx_(std::move(ctx)),
		  isQueryOpened_(true),
		  isAwaitingInput_(true),
		  hasPositiveAnswer_(false),
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
	// this function is called after the reasoning process has been completed,
	// transformedTok represents the response to the query.

	if (transformedTok->indicatesEndOfEvaluation()) {
		activeQueries_.erase(graphQueryIterator);
		// only push EOS message if no query is still active and
		// if the stream has received EOS as input already.
		if (activeQueries_.empty() && !isAwaitingInput_) {
			pushDeferred();
			isQueryOpened_ = false;
			pushToBroadcast(transformedTok);
		}
	} else if (isQueryOpened()) {
		// note that pushing of "no" must be deferred, because it could be that another
		// instance of the query is asked with response "yes" later.

		if (transformedTok->isAnswerToken()) {
			auto answer = std::static_pointer_cast<const Answer>(transformedTok);
			if (answer->isPositive()) {
				hasPositiveAnswer_ = true;
				// if a positive answer is received, all deferred negative answers can be discarded.
				deferredNegativeAnswers_.clear();
				deferredDontKnowAnswers_.clear();
				// directly push any positive response
				pushToBroadcast(transformedTok);
				// close the stage if only one solution is requested
				if ((ctx_->queryFlags & (int) QueryFlag::QUERY_FLAG_ONE_SOLUTION) ==
					(int) QueryFlag::QUERY_FLAG_ONE_SOLUTION) {
					pushToBroadcast(EndOfEvaluation::get());
					close();
				}
			} else if (answer->isNegative()) {
				deferredNegativeAnswers_.emplace_back(std::static_pointer_cast<const AnswerNo>(answer));
			} else {
				deferredDontKnowAnswers_.emplace_back(std::static_pointer_cast<const AnswerDontKnow>(answer));
			}
		} else {
			// push any other token
			pushToBroadcast(transformedTok);
		}
	}
}

void QueryStage::pushDeferred() {
	// all positive answers are pushed directly, only negative answers are deferred.
	// but only push a negative answer if no positive answer has been produced.
	if (!hasPositiveAnswer_) {
		if (!deferredNegativeAnswers_.empty() || deferredDontKnowAnswers_.empty()) {
			if (deferredNegativeAnswers_.size() == 1) {
				pushToBroadcast(deferredNegativeAnswers_.front());
			} else {
				auto no = std::make_shared<AnswerNo>();
				if (deferredNegativeAnswers_.empty()) {
					no->setIsUncertain(true, std::nullopt);
				} else {
					for (auto &x: deferredNegativeAnswers_) {
						no->mergeWith(*x);
					}
				}
				pushToBroadcast(no);
			}
		} else {
			pushToBroadcast(deferredDontKnowAnswers_.front());
		}
	}
	deferredNegativeAnswers_.clear();
	deferredDontKnowAnswers_.clear();
}

void QueryStage::push(const TokenPtr &tok) {
	if (tok->indicatesEndOfEvaluation()) {
		// EOS indicates that no more input is to be expected
		isAwaitingInput_ = false;

		// only broadcast EOS if no graph query is still active.
		if (activeQueries_.empty() && !hasStopRequest_) {
			pushDeferred();
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
			// This assumes "no" is only send by precious stage when no "yes" is expected anymore.
			pushToBroadcast(answer);
		}
	} else {
		// push any other token
		pushToBroadcast(tok);
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
