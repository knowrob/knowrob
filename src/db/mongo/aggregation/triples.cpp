/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <sstream>
#include "knowrob/db/mongo/aggregation/triples.h"
#include "knowrob/semweb/ImportHierarchy.h"
#include "knowrob/Logger.h"
#include "knowrob/terms/Numeric.h"
#include "knowrob/db/mongo/MongoTriplePattern.h"
#include "knowrob/db/mongo/MongoTerm.h"

using namespace knowrob;
using namespace knowrob::mongo;

static inline void matchEmptyArray(Pipeline &pipeline, const char *arrayKey) {
	bson_t emptyArray;
	auto matchStage = pipeline.appendStageBegin("$match");
	BSON_APPEND_ARRAY_BEGIN(matchStage, arrayKey, &emptyArray);
	bson_append_array_end(matchStage, &emptyArray);
	pipeline.appendStageEnd(matchStage);
}

static inline void appendSetVariable(bson_t *doc,
									 const std::string_view &varName,
									 const std::string_view &newValue) {
	const auto varKey = MongoTerm::variableKey(varName);
	const auto varValue = std::string("$") + varKey;
	bson_t condDoc, condArray, notOperator, notArray;
	// Only conditionally apply new value in case it has not been grounded before.
	// varKey: {$cond: [ { $not: [ varValue ] }, newValue, varValue ]}
	BSON_APPEND_DOCUMENT_BEGIN(doc, varKey.c_str(), &condDoc);
	BSON_APPEND_ARRAY_BEGIN(&condDoc, "$cond", &condArray);
	{
		BSON_APPEND_DOCUMENT_BEGIN(&condArray, "0", &notOperator);
		BSON_APPEND_ARRAY_BEGIN(&notOperator, "$not", &notArray);
		BSON_APPEND_UTF8(&notArray, "0", varValue.c_str());
		bson_append_array_end(&notOperator, &notArray);
		bson_append_document_end(&condArray, &notOperator);

		BSON_APPEND_UTF8(&condArray, "1", newValue.data());
		BSON_APPEND_UTF8(&condArray, "2", varValue.c_str());
	}
	bson_append_array_end(&condDoc, &condArray);
	bson_append_document_end(doc, &condDoc);
}

static inline void appendMatchVariable(bson_t *doc,
									   const std::string_view &fieldValue,
									   const std::string_view &varName,
									   const aggregation::TripleLookupData &lookupData) {
	const auto varKey = MongoTerm::variableKey(varName);
	const auto varLetValue = std::string("$$") + varKey;
	const auto matchOperator = (fieldValue.back() == '*' ? "$in" : "$eq");
	bson_t exprDoc, orArr, notDoc, notArr, matchDoc, matchArr;

	BSON_APPEND_DOCUMENT_BEGIN(doc, "$expr", &exprDoc);

	if (lookupData.knownGroundedVariables.count(varName) > 0) {
		// variable is known to have a grounding in the v_VARS field
		// {$expr: { matchOperator: [varLetValue, fieldValue] } }
		BSON_APPEND_ARRAY_BEGIN(&exprDoc, matchOperator, &matchArr);
		BSON_APPEND_UTF8(&matchArr, "0", varLetValue.c_str());
		BSON_APPEND_UTF8(&matchArr, "1", fieldValue.data());
		bson_append_array_end(&exprDoc, &matchArr);
	} else {
		// {$expr: {$or: [ { $not: [ varLetValue ] }, { matchOperator: [varLetValue, fieldValue] } ]}}
		BSON_APPEND_ARRAY_BEGIN(&exprDoc, "$or", &orArr);

		// { $not: [ varLetValue ] }
		BSON_APPEND_DOCUMENT_BEGIN(&orArr, "0", &notDoc);
		BSON_APPEND_ARRAY_BEGIN(&notDoc, "$not", &notArr);
		BSON_APPEND_UTF8(&notArr, "0", varLetValue.c_str());
		bson_append_array_end(&notDoc, &notArr);
		bson_append_document_end(&orArr, &notDoc);

		// { matchOperator: [varLetValue, fieldValue] }
		BSON_APPEND_DOCUMENT_BEGIN(&orArr, "1", &matchDoc);
		BSON_APPEND_ARRAY_BEGIN(&matchDoc, matchOperator, &matchArr);
		BSON_APPEND_UTF8(&matchArr, "0", varLetValue.c_str());
		BSON_APPEND_UTF8(&matchArr, "1", fieldValue.data());
		bson_append_array_end(&matchDoc, &matchArr);
		bson_append_document_end(&orArr, &matchDoc);

		bson_append_array_end(&exprDoc, &orArr);
	}
	bson_append_document_end(doc, &exprDoc);
}

static void setTripleVariables(
		Pipeline &pipeline,
		const aggregation::TripleLookupData &lookupData) {
	std::list<std::pair<const char *, Variable *>> varList;
	for (auto &it: {
			std::make_pair("$next.s", lookupData.expr->subjectTerm()),
			std::make_pair("$next.p", lookupData.expr->propertyTerm()),
			std::make_pair("$next.o", lookupData.expr->objectTerm()),
			std::make_pair("$next.graph", lookupData.expr->graphTerm().get()),
			std::make_pair("$next.confidence", lookupData.expr->confidenceTerm().get()),
			std::make_pair("$next.agent", lookupData.expr->perspectiveTerm().get()),
			std::make_pair("$next.scope.time.since", lookupData.expr->beginTerm().get()),
			std::make_pair("$next.scope.time.until", lookupData.expr->endTerm().get()),
			std::make_pair("$next.uncertain", lookupData.expr->isUncertainTerm().get()),
			std::make_pair("$next.occasional", lookupData.expr->isOccasionalTerm().get())
	}) {
		if (!it.second || it.second->termType() != TermType::VARIABLE) continue;
		auto var = (Variable *) it.second.get();
		// skip variables that were instantiated in previous steps
		if (lookupData.knownGroundedVariables.count(var->name()) > 0) continue;
		varList.emplace_back(it.first, var);
	}

	if (!varList.empty()) {
		auto setVariables = pipeline.appendStageBegin("$set");
		for (auto &it: varList) {
			appendSetVariable(setVariables, it.second->name(), it.first);
		}
		pipeline.appendStageEnd(setVariables);
	}
}

static void nonTransitiveLookup(
		Pipeline &pipeline,
		const TripleStore &tripleStore,
		const aggregation::TripleLookupData &lookupData,
		const semweb::PropertyPtr &definedProperty) {
	bool b_isTaxonomicProperty = (definedProperty &&
								  tripleStore.vocabulary->isTaxonomicProperty(definedProperty->iri()));
	bool b_isReflexiveProperty = (definedProperty &&
								  definedProperty->hasFlag(semweb::PropertyFlag::REFLEXIVE_PROPERTY));
	char arrIndexStr[16];
	const char *arrIndexKey;
	uint32_t arrIndex;

	bool b_skipInputGroundings = false;
	if (lookupData.expr->numVariables() == 0) {
		// the triple expression has no variables
		b_skipInputGroundings = true;
	} else if (!lookupData.mayHasMoreGroundings) {
		// all possible previous instantiations are known and stored in lookupData.knownGroundedVariables.
		b_skipInputGroundings = true;
		for (auto &exprTerm: {
				lookupData.expr->subjectTerm(),
				lookupData.expr->propertyTerm(),
				lookupData.expr->objectTerm()
		}) {
			if (exprTerm->termType() != TermType::VARIABLE) continue;
			auto var = (Variable *) exprTerm.get();
			// skip if this variable cannot have a runtime grounding
			if (lookupData.knownGroundedVariables.count(var->name()) > 0) {
				// the expression contains a variable that is known to have received a grounding before
				b_skipInputGroundings = false;
				break;
			}
		}
	}

	// filter out documents that do not match the triple pattern. this is done using $match or $lookup operators.
	// first lookup matching documents and store them in the 'next' field
	bson_t lookupArray, letDoc;
	auto lookupStage = pipeline.appendStageBegin("$lookup");
	BSON_APPEND_UTF8(lookupStage, "from", tripleStore.tripleCollection->name().data());
	BSON_APPEND_UTF8(lookupStage, "as", "next");
	if (!b_skipInputGroundings) {
		// pass "v_VARS" field to lookup pipeline
		BSON_APPEND_DOCUMENT_BEGIN(lookupStage, "let", &letDoc);
		BSON_APPEND_UTF8(&letDoc, "v_VARS", "$v_VARS");
		bson_append_document_end(lookupStage, &letDoc);
	}
	BSON_APPEND_ARRAY_BEGIN(lookupStage, "pipeline", &lookupArray);
	{
		Pipeline lookupPipeline(&lookupArray);

		auto matchStage = lookupPipeline.appendStageBegin("$match");
		if (b_skipInputGroundings) {
			MongoTriplePattern::append(matchStage, *lookupData.expr, b_isTaxonomicProperty,
									   tripleStore.importHierarchy);
		} else {
			// need to match with potential groundings of variables from previous steps these are stored in the "v_VARS" field.
			bson_t andArray, tripleDoc, variablesDoc;
			BSON_APPEND_ARRAY_BEGIN(matchStage, "$and", &andArray);
			{
				BSON_APPEND_DOCUMENT_BEGIN(&andArray, "0", &tripleDoc);
				MongoTriplePattern::append(&tripleDoc, *lookupData.expr, b_isTaxonomicProperty,
										   tripleStore.importHierarchy);
				bson_append_document_end(&andArray, &tripleDoc);

				// match triple values with previously grounded variables
				arrIndex = 1;
				for (auto &it: {
						std::make_pair("$s", lookupData.expr->subjectTerm()),
						std::make_pair(b_isTaxonomicProperty ? "$p" : "$p*", lookupData.expr->propertyTerm()),
						std::make_pair(b_isTaxonomicProperty ? "$o*" : "$o", lookupData.expr->objectTerm())
				}) {
					if (it.second->termType() != TermType::VARIABLE) continue;
					auto var = (Variable *) it.second.get();
					// skip if this variable cannot have a runtime grounding
					if (!lookupData.mayHasMoreGroundings &&
						lookupData.knownGroundedVariables.count(var->name()) == 0)
						continue;

					bson_uint32_to_string(arrIndex++,
										  &arrIndexKey, arrIndexStr, sizeof arrIndexStr);
					BSON_APPEND_DOCUMENT_BEGIN(&andArray, arrIndexKey, &variablesDoc);
					appendMatchVariable(&variablesDoc, it.first, var->name(), lookupData);
					bson_append_document_end(&andArray, &variablesDoc);
				}
			}
			bson_append_array_end(matchStage, &andArray);
		}
		lookupPipeline.appendStageEnd(matchStage);
		// { $limit: maxNumOfTriples }
		if (lookupData.maxNumOfTriples > 0) lookupPipeline.limit(lookupData.maxNumOfTriples);
	}
	bson_append_array_end(lookupStage, &lookupArray);
	pipeline.appendStageEnd(lookupStage);

	// TODO: add additional results if P is a reflexive property
	if (b_isReflexiveProperty) {
/*
        bson_t nextArray;
        // { $unwind: "$next" }
        pipeline.unwind("$next");
        // { $set: { start: "$next", next: ["$next"] } }
        auto setStage = pipeline.appendStageBegin("$set"); {
            BSON_APPEND_UTF8(setStage, "start", "$next");
            BSON_APPEND_ARRAY_BEGIN(setStage, "next", &nextArray);
            BSON_APPEND_UTF8(&nextArray, "0", "$next");
            bson_append_array_end(setStage, &nextArray);
        }
        pipeline.appendStageEnd(setStage);
        // { $set: { t_refl: {
        //      s: StartValue,
        //      p: "$start.p",
        //      "p*": "$start.p*",
        //      o: StartValue,
        //      "o*": [StartValue],
        //      graph: "$start.graph",
        //      scope: "$start.scope"
        // }}}
        // { $set: { next: { $concatArrays: [["$t_refl"], "$next"] }}}
        // { $unset: ["t_refl","start"] }
        //
*/
	}

	if (lookupData.expr->isNegated()) {
		// following closed-world assumption succeed if no solutions have been found for
		// the formula which appears negated in the queried literal
		matchEmptyArray(pipeline, "next");
	} else {
		// at this point the 'next' field holds an array of matching documents that is unwinded next.
		// TODO: set ignoreEmpty=true if lookupData.expr->isOptional(), but it might be that some of below code will
		//       fail if next has no triple data.
		pipeline.unwind("$next");
		// compute the intersection of time interval so far with time interval of next triple.
		// note that the operations works fine in case the time interval is undefined.
		// TODO: below time interval computation is only ok assuming the statements are not "occasional"
		//intersectTimeInterval(pipeline,
		//					  "$next.scope.time.since",
		//					  "$next.scope.time.until");
		// then verify that the scope is non-empty.
		//matchSinceBeforeUntil(pipeline);
		// remember if one of the statements used to draw the answer is uncertain
		//updateUncertainFlag(pipeline);
		// project new variable groundings
		setTripleVariables(pipeline, lookupData);
	}

	// remove next field again: { $unset: "next" }
	pipeline.unset("next");
}

static void transitiveLookup(
		Pipeline &pipeline,
		const TripleStore &tripleStore,
		const aggregation::TripleLookupData &lookupData,
		const semweb::PropertyPtr &definedProperty) {
	bool b_isReflexiveProperty = (definedProperty &&
								  definedProperty->hasFlag(semweb::PropertyFlag::REFLEXIVE_PROPERTY));
	bool b_isTaxonomicProperty = (definedProperty &&
								  tripleStore.vocabulary->isTaxonomicProperty(definedProperty->iri()));
	// TODO: start with object in case it is known to be grounded before at runtime
	bool b_startWithSubject = lookupData.expr->subjectTerm()->isGround() ||
							  !lookupData.expr->objectTerm()->isGround();

	auto startTerm = (b_startWithSubject ?
					  lookupData.expr->subjectTerm() : lookupData.expr->objectTerm());
	auto endTerm = (b_startWithSubject ?
					lookupData.expr->objectTerm() : lookupData.expr->subjectTerm());

	// recursive lookup
	bson_t restrictSearchDoc;
	auto lookupStage = pipeline.appendStageBegin("$graphLookup");
	BSON_APPEND_UTF8(lookupStage, "from", tripleStore.tripleCollection->name().data());
	if (startTerm->termType() == TermType::ATOMIC) {
		auto startString = (Atomic *) startTerm.get();
		BSON_APPEND_UTF8(lookupStage, "startWith", startString->stringForm().data());
	} else if (startTerm->termType() == TermType::VARIABLE) {
		auto startVariable = (Variable *) startTerm.get();
		auto startValue = std::string("$") + MongoTerm::variableKey(startVariable->name());
		BSON_APPEND_UTF8(lookupStage, "startWith", startValue.c_str());
	} else {
		KB_WARN("Ignoring term {} with invalid type for graph lookup.", *startTerm);
	}
	BSON_APPEND_UTF8(lookupStage, "connectToField", b_startWithSubject ? "s" : "o");
	BSON_APPEND_UTF8(lookupStage, "connectFromField", b_startWithSubject ? "p" : "s");
	BSON_APPEND_UTF8(lookupStage, "as", "t_paths");
	BSON_APPEND_UTF8(lookupStage, "depthField", "depth");
	/* { restrictSearchWithMatch: { p*: Query_p, ... }" */
	BSON_APPEND_DOCUMENT_BEGIN(lookupStage, "restrictSearchWithMatch", &restrictSearchDoc);
	{
		MongoTerm::append(&restrictSearchDoc,
						  (b_isTaxonomicProperty ? "p" : "p*"),
						  lookupData.expr->propertyTerm());
		MongoTriplePattern::appendGraphSelector(&restrictSearchDoc, *lookupData.expr, tripleStore.importHierarchy);
		MongoTriplePattern::appendEpistemicSelector(&restrictSearchDoc, *lookupData.expr);
		MongoTriplePattern::appendTimeSelector(&restrictSearchDoc, *lookupData.expr);
	}
	bson_append_document_end(lookupStage, &restrictSearchDoc);
	pipeline.appendStageEnd(lookupStage);

	// $graphLookup does not ensure order, so we need to order by recursion depth in a separate step
	// TODO: mongo v5.2 has $sortArray that could replace below $lookup
	bson_t letDoc, orderingArray;
	auto orderingStage = pipeline.appendStageBegin("$lookup");
	BSON_APPEND_UTF8(orderingStage, "from", "one");
	BSON_APPEND_UTF8(orderingStage, "as", "t_sorted");
	// pass "t_paths" field to lookup pipeline
	BSON_APPEND_DOCUMENT_BEGIN(orderingStage, "let", &letDoc);
	BSON_APPEND_UTF8(&letDoc, "t_paths", "$t_paths");
	bson_append_document_end(orderingStage, &letDoc);
	BSON_APPEND_ARRAY_BEGIN(orderingStage, "pipeline", &orderingArray);
	{
		Pipeline orderingPipeline(&orderingArray);
		// { $set: { t_paths: "$$t_paths") } }
		auto setStage = orderingPipeline.appendStageBegin("$set");
		BSON_APPEND_UTF8(orderingStage, "t_paths", "$$t_paths");
		orderingPipeline.appendStageEnd(setStage);
		// { $unwind: $t_paths }
		orderingPipeline.unwind("$t_paths");
		// { $replaceRoot: { newRoot: "$t_paths" } }
		orderingPipeline.replaceRoot("$t_paths");
		// { $sort: { depth: 1 } }
		orderingPipeline.sortAscending("depth");
	}
	bson_append_document_end(orderingStage, &orderingArray);
	pipeline.appendStageEnd(orderingStage);

	// { $set: { next: "$t_sorted" } }
	auto setNext = pipeline.appendStageBegin("$set");
	BSON_APPEND_UTF8(setNext, "next", "$t_sorted");
	pipeline.appendStageEnd(setNext);

	// { $set: { start: { $arrayElemAt: ["$next", 0] } } }
	bson_t setStartOperation, setStartArr;
	auto setStart = pipeline.appendStageBegin("$set");
	BSON_APPEND_DOCUMENT_BEGIN(setStart, "start", &setStartOperation);
	BSON_APPEND_ARRAY_BEGIN(&setStartOperation, "$arrayElemAt", &setStartArr);
	BSON_APPEND_UTF8(&setStartArr, "0", "$next");
	BSON_APPEND_INT32(&setStartArr, "1", 0);
	bson_append_array_end(&setStartOperation, &setStartArr);
	bson_append_document_end(setStart, &setStartOperation);
	pipeline.appendStageEnd(setStart);

	// { $unset: "t_paths" }, { $unset: "t_sorted" }
	pipeline.unset("t_paths");
	pipeline.unset("t_sorted");

	if (b_isReflexiveProperty) {
		// TODO: handle reflexivity in triple graph lookup
		// reflexivity(StartValue, Ctx, Step)
	}

	// iterate over results: { $unwind: "$next" }
	pipeline.unwind("$next");

	// graph lookup uses "s" or "o" as start for recursive lookup but ignores the other.
	// thus a matching must be performed for the results.
	if (endTerm->termType() != TermType::VARIABLE) {
		// FIXME: must add another match case for endTerm being a runtime grounded variable.
		bson_t matchEndVal;
		auto matchEnd = pipeline.appendStageBegin("$match");
		BSON_APPEND_DOCUMENT_BEGIN(matchEnd, "next.o", &matchEndVal);
		MongoTerm::append(&restrictSearchDoc,
						  (b_startWithSubject ? "next.o" : "next.s"),
						  endTerm);
		bson_append_document_end(matchEnd, &matchEndVal);
		pipeline.appendStageEnd(matchEnd);
	}

	// compute the intersection of time interval so far with time interval of next triple.
	// note that the operations work fine in case the time interval is undefined.
	// FIXME: intersection need to be performed over all transitions in graph lookup
	//intersectTimeInterval(pipeline,
	//					  "$next.scope.time.since",
	//					  "$next.scope.time.until");
	// then verify that the scope is non-empty.
	//matchSinceBeforeUntil(pipeline);
	// project new variable groundings
	setTripleVariables(pipeline, lookupData);
	// remove next field again: { $unset: "next" }
	pipeline.unset("next");
}

void aggregation::lookupTriple(Pipeline &pipeline, const TripleStore &tripleStore, const TripleLookupData &lookupData) {
	semweb::PropertyPtr definedProperty;
	if (lookupData.expr->propertyTerm() && lookupData.expr->propertyTerm()->termType() == TermType::ATOMIC) {
		auto propertyTerm = std::static_pointer_cast<Atomic>(lookupData.expr->propertyTerm());
		definedProperty = tripleStore.vocabulary->getDefinedProperty(propertyTerm->stringForm());
	}

	bool b_isTransitiveProperty = (definedProperty && definedProperty->hasFlag(
			semweb::PropertyFlag::TRANSITIVE_PROPERTY));
	if (b_isTransitiveProperty || lookupData.forceTransitiveLookup)
		transitiveLookup(pipeline, tripleStore, lookupData, definedProperty);
	else
		nonTransitiveLookup(pipeline, tripleStore, lookupData, definedProperty);
}
