/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_BSON_H
#define KNOWROB_BSON_H

#include <mongoc.h>
#include <optional>

namespace knowrob::mongo {
	/**
	 * A wrapper for bson_t used to avoid warnings reported by
	 * gcc about using bson_t as a template argument.
	 */
	typedef struct {
		bson_t bson;
	} bson_wrapper;

	/**
	 * A wrapper for bson_t used to avoid warnings reported by
	 * gcc about using bson_t as a template argument.
	 */
	typedef struct {
		const bson_t *bson;
	} bson_wrapper_ptr;

	/**
	 * @param iter a bson iterator
	 * @return the double value of iter or null opt if none.
	 */
	std::optional<double> bson_iterOptionalDouble(const bson_iter_t *iter);
}

#endif //KNOWROB_BSON_H
