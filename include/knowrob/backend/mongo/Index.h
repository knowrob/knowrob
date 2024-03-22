/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGO_INDEX_H
#define KNOWROB_MONGO_INDEX_H

namespace knowrob::mongo {
	/**
	 * The type of an index.
	 */
	enum class IndexType {
		ASCENDING = 1,
		DESCENDING = -1,
	};

	/**
	 * Configures an index key.
	 */
	struct IndexKey {
		explicit IndexKey(const char *key, IndexType type = IndexType::ASCENDING)
				: value(key), type(type) {};
		const std::string value;
		const IndexType type;
	};
}

#endif //KNOWROB_MONGO_INDEX_H
