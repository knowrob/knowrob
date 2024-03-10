/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGO_TRIPLE_H
#define KNOWROB_MONGO_TRIPLE_H

#include "knowrob/semweb/Vocabulary.h"
#include "knowrob/triples/FramedTriple.h"
#include "Document.h"

namespace knowrob::mongo {
	/**
	 * Transforms a triple into a MongoDB document.
	 */
	class MongoTriple {
	public:
		MongoTriple(const semweb::VocabularyPtr &vocabulary,
					const FramedTriple &tripleData,
					const std::string &fallbackOrigin,
					bool isTaxonomic);

		/**
		 * @return the document.
		 */
		auto &document() const { return document_; }

	protected:
		Document document_;

		static bson_t *createDocument(
				const semweb::VocabularyPtr &vocabulary,
				const FramedTriple &tripleData,
				const std::string &fallbackOrigin,
				bool isTaxonomic);
	};

} // knowrob

#endif //KNOWROB_MONGO_TRIPLE_H
