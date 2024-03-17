/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGO_BINDINGS_CURSOR_H
#define KNOWROB_MONGO_BINDINGS_CURSOR_H

#include "Cursor.h"
#include "knowrob/terms/Bindings.h"

namespace knowrob::mongo {
	/**
	 * A getAnswerCursor that iterates over bindings computed through an aggregation
	 * pipeline which uses a special field in documents to store variable
	 * bindings throughout the pipeline.
	 */
	class BindingsCursor : public Cursor {
	public:
		explicit BindingsCursor(const std::shared_ptr<Collection> &collection);

		bool nextBindings(const std::shared_ptr<Bindings> &bindings);

	protected:
		const bson_t *resultDocument_;
		bson_iter_t resultIter_;
		bson_iter_t varIter_;
		bson_iter_t valIter_;

		void setSubstitution(const std::shared_ptr<Bindings> &bindings);
	};

	using BindingsCursorPtr = std::shared_ptr<BindingsCursor>;
} // mongo

#endif //KNOWROB_MONGO_BINDINGS_CURSOR_H
