/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/db/SPARQLBackend.h"

using namespace knowrob;

bool SPARQLBackend::query(const SPARQLQuery &q, const BindingsHandler &callback) const {
	return sparql(q(), callback);
}

void SPARQLBackend::query(const GraphQueryPtr &q, const BindingsHandler &callback) { // NOLINT
	std::shared_ptr<GraphPattern> negatedPattern;
	if (q->term()->termType() == GraphTermType::Pattern) {
		// Currently we cannot support negations where the object is grounded.
		// This is because the SPARQL query would need to use NOT-EXISTS or MINUS, which is not supported by rasqal.
		// Instead, we use OPTIONAL and !BOUND to simulate negation which requires that the object is not grounded
		// when entering the negated pattern.
		// But, on the other hand negations are handled separately by the KnowledgeBase anyway.
		// So KnowRob will at the moment only call this interface with single negated patterns
		// while evaluating sequences of negations in parallel and after any positive pattern that
		// appears in a query.
		auto pat = std::static_pointer_cast<GraphPattern>(q->term());
		if (pat->value()->isNegated()) {
			negatedPattern = pat;
		}
	}

	if (negatedPattern) {
		// negation-as-failure: Try positive query, if it has no solution, then the negated pattern is true.
		auto positivePat = std::make_shared<FramedTriplePattern>(*negatedPattern->value());
		positivePat->setIsNegated(false);
		auto positiveQuery = std::make_shared<GraphQuery>(
				std::make_shared<GraphPattern>(positivePat),
				OneSolutionContext());
		bool hasSolution = false;
		query(positiveQuery, [&](const BindingsPtr &bindings) {
			hasSolution = true;
		});
		if (!hasSolution) {
			callback(Bindings::emptyBindings());
		}
	} else {
		sparql(SPARQLQuery(q, sparqlFlags_)(), callback);
	}
}

void SPARQLBackend::count(const ResourceCounter &callback) const {
	static const char *sparqlString = "SELECT ?resource (COUNT(?s) AS ?count) WHERE "\
                "{ ?s rdf:type ?resource . } UNION { ?s ?resource ?o . } } "\
                "GROUP BY ?resource";
	sparql(sparqlString, [&](const BindingsPtr &bindings) {
		auto resource = bindings->getAtomic("resource");
		auto count = bindings->getAtomic("count");
		if (!resource || !count || !count->isNumeric()) {
			KB_WARN("Failed to count triples!");
			return;
		}
		callback(resource->stringForm(), std::static_pointer_cast<Numeric>(count)->asLong());
	});
}
