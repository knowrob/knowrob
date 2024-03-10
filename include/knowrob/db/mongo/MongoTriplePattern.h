/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGO_TRIPLE_PATTERN_H
#define KNOWROB_MONGO_TRIPLE_PATTERN_H

#include <mongoc.h>
#include "knowrob/triples/FramedTriplePattern.h"
#include "knowrob/semweb/ImportHierarchy.h"
#include "Document.h"

namespace knowrob {
	/**
	 * A class to represent a triple pattern in a MongoDB.
	 */
	class MongoTriplePattern {
	public:
		MongoTriplePattern(
				const FramedTriplePattern &tripleExpression,
				bool b_isTaxonomicProperty,
				const std::shared_ptr<semweb::ImportHierarchy> &importHierarchy);

		auto &document() { return document_; }

		auto bson() { return document_.bson(); }

		static void append(
				bson_t *selectorDoc,
				const FramedTriplePattern &tripleExpression,
				bool b_isTaxonomicProperty,
				const std::shared_ptr<semweb::ImportHierarchy> &importHierarchy);

		static void appendGraphSelector(
				bson_t *selectorDoc,
				const FramedTriplePattern &tripleExpression,
				const std::shared_ptr<semweb::ImportHierarchy> &importHierarchy);

		static void appendEpistemicSelector(
				bson_t *selectorDoc,
				const FramedTriplePattern &tripleExpression);

		static void appendTimeSelector(
				bson_t *selectorDoc,
				const FramedTriplePattern &tripleExpression);

	protected:
		mongo::Document document_;

		static bson_t *create(
				const FramedTriplePattern &tripleExpression,
				bool b_isTaxonomicProperty,
				const std::shared_ptr<semweb::ImportHierarchy> &importHierarchy);

		static const char *getOperatorString(knowrob::FramedTriplePattern::OperatorType operatorType);
	};

} // knowrob

#endif //KNOWROB_MONGO_TRIPLE_PATTERN_H
