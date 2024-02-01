/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_ANSWER_MERGER_H
#define KNOWROB_ANSWER_MERGER_H

#include "AnswerTransformer.h"
#include "AnswerYes.h"

namespace knowrob {
	/**
	 * A transformer that merges two partial answers.
	 */
	class AnswerMerger : public AnswerTransformer {
	public:
		explicit AnswerMerger(AnswerPtr partialResult);

		// override AnswerTransformer
		TokenPtr transform(const TokenPtr &tok) override;

	protected:
		const AnswerPtr partialResult_;
	};

} // knowrob

#endif //KNOWROB_ANSWER_MERGER_Hs
