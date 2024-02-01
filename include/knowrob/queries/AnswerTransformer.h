//
// Created by daniel on 22.04.23.
//

#ifndef KNOWROB_ANSWER_TRANSFORMER_H
#define KNOWROB_ANSWER_TRANSFORMER_H

#include <knowrob/queries/Answer.h>
#include <knowrob/queries/TokenBroadcaster.h>

#include <utility>

namespace knowrob {
	/**
	 * Broadcasts each input message after applying a transformation.
	 */
	class AnswerTransformer : public TokenBroadcaster {
	public:
		explicit AnswerTransformer() : TokenBroadcaster() {}

		/**
		 * Transform a token.
		 * @param tok the token to transform.
		 * @return the transformed token.
		 */
		virtual TokenPtr transform(const TokenPtr &tok) = 0;

	protected:

		// Override AnswerStream
		void push(const TokenPtr &tok) override { TokenBroadcaster::push(transform(tok)); }
	};

} // knowrob

#endif //KNOWROB_ANSWER_TRANSFORMER_H
