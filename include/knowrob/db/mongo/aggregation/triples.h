/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_MONGO_AGGREGATION_TRIPLES_H
#define KNOWROB_MONGO_AGGREGATION_TRIPLES_H

#include <mongoc.h>
#include <string>
#include <optional>
#include "knowrob/terms/Term.h"
#include "knowrob/db/mongo/Pipeline.h"
#include "knowrob/semweb/ImportHierarchy.h"
#include "knowrob/semweb/Vocabulary.h"
#include "knowrob/triples/FramedTriplePattern.h"

namespace knowrob::mongo::aggregation {
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
}

#endif //KNOWROB_MONGO_AGGREGATION_TRIPLES_H
