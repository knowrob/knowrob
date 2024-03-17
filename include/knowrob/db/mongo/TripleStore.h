/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGO_TRIPLE_STORE_H
#define KNOWROB_MONGO_TRIPLE_STORE_H

#include <memory>
#include <utility>
#include "Collection.h"
#include "knowrob/semweb/Vocabulary.h"
#include "knowrob/semweb/ImportHierarchy.h"

namespace knowrob::mongo {
	/**
	 * Encapsulates common objects when working with triples.
	 */
	struct TripleStore {
		TripleStore(std::shared_ptr<knowrob::mongo::Collection> tripleCollection,
					std::shared_ptr<knowrob::mongo::Collection> oneCollection,
					std::shared_ptr<knowrob::Vocabulary> vocabulary)
				: tripleCollection(std::move(tripleCollection)),
				  oneCollection(std::move(oneCollection)),
				  vocabulary(std::move(vocabulary)) {}

		std::shared_ptr<knowrob::mongo::Collection> tripleCollection;
		std::shared_ptr<knowrob::mongo::Collection> oneCollection;
		std::shared_ptr<knowrob::Vocabulary> vocabulary;
	};
}

#endif //KNOWROB_MONGO_TRIPLE_STORE_H
