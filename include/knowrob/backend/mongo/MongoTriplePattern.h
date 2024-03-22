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
#include "Pipeline.h"
#include "TripleStore.h"

namespace knowrob::mongo {
	/**
	 * A class to represent a triple pattern in a MongoDB.
	 */
	class MongoTriplePattern {
	public:
		MongoTriplePattern(
				const FramedTriplePattern &tripleExpression,
				bool b_isTaxonomicProperty,
				const std::shared_ptr<ImportHierarchy> &importHierarchy);

		auto &document() { return document_; }

		auto bson() { return document_.bson(); }

		static void append(
				bson_t *selectorDoc,
				const FramedTriplePattern &tripleExpression,
				bool b_isTaxonomicProperty,
				const std::shared_ptr<ImportHierarchy> &importHierarchy);

		static void appendGraphSelector(
				bson_t *selectorDoc,
				const FramedTriplePattern &tripleExpression,
				const std::shared_ptr<ImportHierarchy> &importHierarchy);

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
				const std::shared_ptr<ImportHierarchy> &importHierarchy);

		static const char *getOperatorString(knowrob::FilterType operatorType);
	};

	struct TripleLookupData {
		explicit TripleLookupData(const FramedTriplePattern *expr)
				: expr(expr),
				  maxNumOfTriples(0),
				  mayHasMoreGroundings(true),
				  forceTransitiveLookup(false) {}

		const FramedTriplePattern *expr;
		uint32_t maxNumOfTriples;
		std::set<std::string_view> knownGroundedVariables;
		bool mayHasMoreGroundings;
		bool forceTransitiveLookup;
	};

	void lookupTriple(Pipeline &pipeline, const TripleStore &tripleStore, const TripleLookupData &lookupData);

} // knowrob

#endif //KNOWROB_MONGO_TRIPLE_PATTERN_H
