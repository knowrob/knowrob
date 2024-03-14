/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGO_DOCUMENT_H
#define KNOWROB_MONGO_DOCUMENT_H

#include <mongoc.h>

namespace knowrob::mongo {
	/**
	 * A scoped bson document. The memory of the bson_t* is managed by this
	 * object, and is freed when the object is destroyed.
	 */
	class Document {
	public:
		explicit Document(bson_t *bson) : bson_(bson) {}

		Document(const Document &) = delete;

		~Document() { bson_destroy(bson_); }

		/**
		 * @return the managed bson document
		 */
		bson_t *bson() const { return bson_; }

		static void print_canonical(bson_t *bson) {
			size_t len;
			auto str = bson_as_canonical_extended_json(bson, &len);
			printf("%s\n", str);
			bson_free(str);
		}

		static void print_relaxed(bson_t *bson) {
			size_t len;
			auto str = bson_as_relaxed_extended_json(bson, &len);
			printf("%s\n", str);
			bson_free(str);
		}

	protected:
		bson_t *bson_;
	};

} // knowrob

#endif //KNOWROB_MONGO_DOCUMENT_H
