/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGO_BINDINGS_CURSOR_H
#define KNOWROB_MONGO_BINDINGS_CURSOR_H

#include "Cursor.h"
#include "knowrob/queries/Answer.h"
#include "knowrob/queries/AnswerYes.h"
#include "knowrob/queries/FramedBindings.h"
#include "knowrob/triples/FramedTriplePattern.h"

namespace knowrob::mongo {
	class BindingsCursor : public Cursor {
	public:
		explicit BindingsCursor(const std::shared_ptr<Collection> &collection);

		bool nextBindings(const FramedBindingsPtr &bindings);

	protected:
		const bson_t *resultDocument_;
		bson_iter_t resultIter_;
		bson_iter_t varIter_;
		bson_iter_t valIter_;
		bson_iter_t scopeIter_;
		bson_iter_t timeIter_;

		void setSubstitution(const FramedBindingsPtr &bindings);

		std::shared_ptr<GraphSelector> readAnswerFrame();
	};

	using BindingsCursorPtr = std::shared_ptr<BindingsCursor>;
} // mongo

#endif //KNOWROB_MONGO_BINDINGS_CURSOR_H
