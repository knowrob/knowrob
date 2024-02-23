//
// Created by daniel on 08.04.23.
//

#ifndef KNOWROB_MONGO_ANSWER_CURSOR_H
#define KNOWROB_MONGO_ANSWER_CURSOR_H

#include "Cursor.h"
#include "knowrob/queries/Answer.h"
#include "knowrob/queries/AnswerYes.h"
#include "knowrob/semweb/FramedTriplePattern.h"

namespace knowrob::mongo {
	/**
	 * A mongo cursor tht generates Answer objects.
	 * It is assumed that all grounding of free variables are recorded
	 * in output documents in a dedicated field.
	 */
	class AnswerCursor : public Cursor {
	public:
		explicit AnswerCursor(const std::shared_ptr<Collection> &collection);

		/**
		 * Pull the next answer from this cursor.
		 * @param answer the answer
		 * @return true on success, false indicates that the cursor is empty
		 */
		bool nextAnswer(const std::shared_ptr<AnswerYes> &answer, const std::vector<RDFLiteralPtr> &literals);

	protected:
		const bson_t *resultDocument_;
		bson_iter_t resultIter_;
		bson_iter_t varIter_;
		bson_iter_t valIter_;
		bson_iter_t scopeIter_;
		bson_iter_t timeIter_;

		void setSubstitution(const std::shared_ptr<AnswerYes> &answer);

		std::shared_ptr<GraphSelector> readAnswerFrame(const std::shared_ptr<AnswerYes> &answer);
	};

	using AnswerCursorPtr = std::shared_ptr<AnswerCursor>;
} // mongo

#endif //KNOWROB_ANSWERCURSOR_H
