//
// Created by daniel on 08.04.23.
//

#ifndef KNOWROB_MONGO_AGGREGATION_TRIPLES_H
#define KNOWROB_MONGO_AGGREGATION_TRIPLES_H

#include <mongoc.h>
#include <string>
#include <optional>
#include "knowrob/terms/Term.h"
#include "knowrob/db/mongo/Pipeline.h"
#include "knowrob/semweb/ImportHierarchy.h"
#include "knowrob/semweb/Vocabulary.h"
#include "knowrob/semweb/RDFLiteral.h"

namespace knowrob::mongo::aggregation {
	struct TripleLookupData {
		explicit TripleLookupData(const RDFLiteral *expr)
				: expr(expr),
				  maxNumOfTriples(0),
				  mayHasMoreGroundings(true),
				  forceTransitiveLookup(false) {}

		const RDFLiteral *expr;
		uint32_t maxNumOfTriples;
		std::set<std::string_view> knownGroundedVariables;
		bool mayHasMoreGroundings;
		bool forceTransitiveLookup;
	};

	void appendTripleSelector(
			bson_t *selectorDoc,
			const RDFLiteral &tripleExpression,
			bool b_isTaxonomicProperty,
			const std::shared_ptr<semweb::ImportHierarchy> &importHierarchy);

	void appendGraphSelector(
			bson_t *selectorDoc,
			const RDFLiteral &tripleExpression,
			const std::shared_ptr<semweb::ImportHierarchy> &importHierarchy);

	void appendEpistemicSelector(
			bson_t *selectorDoc,
			const RDFLiteral &tripleExpression);

	void appendTimeSelector(
			bson_t *selectorDoc,
			const RDFLiteral &tripleExpression);

	void lookupTriple(
			aggregation::Pipeline &pipeline,
			const std::string_view &collection,
			const std::shared_ptr<semweb::Vocabulary> &vocabulary,
			const std::shared_ptr<semweb::ImportHierarchy> &importHierarchy,
			const TripleLookupData &lookupData);

	void lookupTriplePaths(
			aggregation::Pipeline &pipeline,
			const std::string_view &collection,
			const std::shared_ptr<semweb::Vocabulary> &vocabulary,
			const std::shared_ptr<semweb::ImportHierarchy> &importHierarchy,
			const std::vector<RDFLiteralPtr> &tripleExpressions);
}

#endif //KNOWROB_MONGO_AGGREGATION_TRIPLES_H
