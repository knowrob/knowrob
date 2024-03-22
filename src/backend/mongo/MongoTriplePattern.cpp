/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/backend/mongo/MongoTriplePattern.h"
#include "knowrob/backend/mongo/MongoTerm.h"
#include "knowrob/backend/mongo/Pipeline.h"

using namespace knowrob;
using namespace knowrob::mongo;

#define MONGO_OPERATOR_LTE  "$lte"
#define MONGO_OPERATOR_GTE  "$gte"
#define MONGO_OPERATOR_LT   "$lt"
#define MONGO_OPERATOR_GT   "$gt"
#define MONGO_OPERATOR_NEQ  "$ne"

MongoTriplePattern::MongoTriplePattern(
		const FramedTriplePattern &tripleExpression,
		bool b_isTaxonomicProperty,
		const std::shared_ptr<ImportHierarchy> &importHierarchy)
		: document_(create(tripleExpression, b_isTaxonomicProperty, importHierarchy)) {
}

bson_t *MongoTriplePattern::create(
		const FramedTriplePattern &tripleExpression,
		bool b_isTaxonomicProperty,
		const std::shared_ptr<ImportHierarchy> &importHierarchy) {
	auto selectorDoc = bson_new();
	append(selectorDoc, tripleExpression, b_isTaxonomicProperty, importHierarchy);
	return selectorDoc;
}

void MongoTriplePattern::append(bson_t *selectorDoc,
								const FramedTriplePattern &tripleExpression,
								bool b_isTaxonomicProperty,
								const std::shared_ptr<ImportHierarchy> &importHierarchy) {
	// "s"" field
	MongoTerm::append(selectorDoc,
					  "s", tripleExpression.subjectTerm());
	// "p" field
	MongoTerm::append(selectorDoc,
					  (b_isTaxonomicProperty ? "p" : "p*"),
					  tripleExpression.propertyTerm());
	// "o" field
	const char *objectOperator = getOperatorString(tripleExpression.objectOperator());
	MongoTerm::append(selectorDoc,
					  (b_isTaxonomicProperty ? "o*" : "o"),
					  tripleExpression.objectTerm(),
					  objectOperator);
	// "g" field
	appendGraphSelector(selectorDoc, tripleExpression, importHierarchy);
	// epistemic fields
	appendEpistemicSelector(selectorDoc, tripleExpression);
	// temporal fields
	appendTimeSelector(selectorDoc, tripleExpression);
}

void MongoTriplePattern::appendGraphSelector(bson_t *selectorDoc,
											 const FramedTriplePattern &tripleExpression,
											 const std::shared_ptr<ImportHierarchy> &importHierarchy) {
	auto &gt = tripleExpression.graphTerm();
	if (!gt) return;

	if (gt->termType() == TermType::ATOMIC) {
		auto graphString = gt.grounded();
		if (graphString->stringForm() == ImportHierarchy::ORIGIN_ANY || graphString->stringForm() == "*")
			return;

		std::vector<TermPtr> childrenNames;
		for (auto &child: importHierarchy->getImports(graphString->stringForm())) {
			childrenNames.push_back(Atom::Tabled(child->name()));
		}
		if (childrenNames.empty()) {
			MongoTerm::append(selectorDoc, "graph", graphString);
		} else {
			childrenNames.push_back(graphString);
			MongoTerm::append(selectorDoc, "graph", childrenNames, "$in");
		}
	}
		/*
		else if (gt->termType() == TermType::FUNCTION) {
			auto fn = (Function*) gt.get();
			if (*fn->functor() == *ListTerm::listFunctor()) {
				aggregation::appendArrayQuery(selectorDoc, "graph", fn->arguments(), "$in");
			} else {
				KB_WARN("graph term {} has unexpected function type", *gt);
			}
		}
		 */
	else {
		KB_WARN("graph term {} has unexpected type", *gt);
	}
}

void MongoTriplePattern::appendEpistemicSelector(bson_t *selectorDoc, const FramedTriplePattern &tripleExpression) {
	static const bool allowConfidenceNullValues = true;
	auto &ct = tripleExpression.confidenceTerm();
	auto &at = tripleExpression.perspectiveTerm();
	auto &u = tripleExpression.isUncertainTerm();

	if (!((u.has_grounding() && u.grounded()->asBoolean()) || u.has_variable())) {
		// note: null value of "uncertain" field is seen as value "false"
		MongoTerm::append(
				selectorDoc,
				"uncertain",
				Numeric::falseAtom(),
				nullptr,
				true);
	}

	if (at) {
		if (at->termType() == TermType::ATOMIC) {
			MongoTerm::append(
					selectorDoc,
					"agent",
					*at,
					nullptr,
					false);
		} else {
			KB_WARN("agent term {} has unexpected type", *at);
		}
	} else {
		// make sure agent field is undefined: { agent: { $exists: false } }
		// note: null value of agent field is seen as "self", i.e. the agent running the knowledge base
		bson_t agentDoc;
		BSON_APPEND_DOCUMENT_BEGIN(selectorDoc, "agent", &agentDoc);
		BSON_APPEND_BOOL(&agentDoc, "$exists", false);
		bson_append_document_end(selectorDoc, &agentDoc);
	}

	if (ct) {
		if (ct->termType() == TermType::ATOMIC) {
			// note: null value of confidence is seen as larger than the requested threshold
			MongoTerm::append(
					selectorDoc,
					"confidence",
					*ct,
					MONGO_OPERATOR_GTE,
					true);
		} else {
			KB_WARN("confidence term {} has unexpected type", *ct);
		}
	}
}

void MongoTriplePattern::appendTimeSelector(bson_t *selectorDoc, const FramedTriplePattern &tripleExpression) {
	static const bool allowNullValues = true;
	static auto b_occasional = std::make_shared<Integer>(static_cast<int32_t>(true));
	static auto b_always = std::make_shared<Integer>(static_cast<int32_t>(false));
	auto bt = &tripleExpression.beginTerm();
	auto et = &tripleExpression.endTerm();
	auto &o = tripleExpression.isOccasionalTerm();

	if (!((o.has_grounding() && o.grounded()->asBoolean()) || o.has_variable())) {
		// note: null value of "occasional" field is seen as value "false"
		MongoTerm::append(
				selectorDoc,
				"occasional",
				Numeric::falseAtom(),
				nullptr,
				true);
	} else {
		auto swap = bt;
		bt = et;
		et = swap;
	}

	if (bt->has_grounding()) {
		MongoTerm::append(
				selectorDoc,
				"scope.time.since",
				bt->get(),
				MONGO_OPERATOR_LTE,
				allowNullValues);
	}
	if (et->has_grounding()) {
		MongoTerm::append(
				selectorDoc,
				"scope.time.until",
				et->get(),
				MONGO_OPERATOR_GTE,
				allowNullValues);
	}
}

const char *MongoTriplePattern::getOperatorString(knowrob::FilterType operatorType) {
	switch (operatorType) {
		case FilterType::EQ:
			return nullptr;
		case FilterType::NEQ:
			return MONGO_OPERATOR_NEQ;
		case FilterType::LEQ:
			return MONGO_OPERATOR_LTE;
		case FilterType::GEQ:
			return MONGO_OPERATOR_GTE;
		case FilterType::LT:
			return MONGO_OPERATOR_LT;
		case FilterType::GT:
			return MONGO_OPERATOR_GT;
	}
	return nullptr;
}

static inline void appendSetVariable(bson_t *doc, std::string_view varName, std::string_view newValue) {
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
									   std::string_view fieldValue,
									   std::string_view varName,
									   const TripleLookupData &lookupData) {
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

static void setTripleVariables(Pipeline &pipeline, const TripleLookupData &lookupData) {
	std::list<std::pair<const char *, Variable *>> varList;

	// the object can be specified as a triple (Value, Operator, ActualValue) such that
	// a value constraint is given plus a variable where the actual value of the triple should be stored.
	TermPtr objectVar = lookupData.expr->objectTerm();
	if (objectVar && !objectVar->isVariable() && lookupData.expr->objectVariable()) {
		objectVar = lookupData.expr->objectVariable();
	}

	for (auto &it: {
			std::make_pair("$next.s", lookupData.expr->subjectTerm()),
			std::make_pair("$next.p", lookupData.expr->propertyTerm()),
			std::make_pair("$next.o", objectVar),
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
		const TripleLookupData &lookupData,
		const semweb::PropertyPtr &definedProperty) {
	bool b_isTaxonomicProperty = (definedProperty &&
								  tripleStore.vocabulary->isTaxonomicProperty(definedProperty->iri()));
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
									   tripleStore.vocabulary->importHierarchy());
		} else {
			// need to match with potential groundings of variables from previous steps these are stored in the "v_VARS" field.
			bson_t andArray, tripleDoc, variablesDoc;
			BSON_APPEND_ARRAY_BEGIN(matchStage, "$and", &andArray);
			{
				BSON_APPEND_DOCUMENT_BEGIN(&andArray, "0", &tripleDoc);
				MongoTriplePattern::append(&tripleDoc, *lookupData.expr, b_isTaxonomicProperty,
										   tripleStore.vocabulary->importHierarchy());
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

	// at this point the 'next' field holds an array of matching documents, if any.
	if (lookupData.expr->isNegated()) {
		// following closed-world assumption succeed if no solutions are found
		pipeline.matchEmptyArray("next");
	} else {
		// Unwind, and set preserveEmpty=true if lookupData.expr->isOptional().
		pipeline.unwind("$next", lookupData.expr->isOptional());
		// project new variable groundings
		setTripleVariables(pipeline, lookupData);
	}

	// finally, remove next field: { $unset: "next" }
	pipeline.unset("next");
}

static void transitiveLookup(
		Pipeline &pipeline,
		const TripleStore &tripleStore,
		const TripleLookupData &lookupData,
		const semweb::PropertyPtr &definedProperty) {
	bool b_isTaxonomicProperty = (definedProperty &&
								  tripleStore.vocabulary->isTaxonomicProperty(definedProperty->iri()));
	// start with object in case it is known to be grounded before at runtime
	bool b_objectIsKnownGrounded = lookupData.expr->objectTerm()->isVariable() &&
			lookupData.knownGroundedVariables.count(std::static_pointer_cast<Variable>(lookupData.expr->objectTerm())->name()) > 0;
	bool b_startWithSubject = lookupData.expr->subjectTerm()->isGround() || !b_objectIsKnownGrounded;

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
		MongoTriplePattern::appendGraphSelector(&restrictSearchDoc, *lookupData.expr, tripleStore.vocabulary->importHierarchy());
		MongoTriplePattern::appendEpistemicSelector(&restrictSearchDoc, *lookupData.expr);
		MongoTriplePattern::appendTimeSelector(&restrictSearchDoc, *lookupData.expr);
	}
	bson_append_document_end(lookupStage, &restrictSearchDoc);
	pipeline.appendStageEnd(lookupStage);

	// $graphLookup does not ensure order, so we need to order by recursion depth in a separate step
	// t_sorted = { $sortArray: { input: "$t_paths", sort: { depth: 1 } } }
	bson_t setSortedDoc, sortArrayDoc, sortByDoc;
	auto setSortedStage = pipeline.appendStageBegin("$set");
	BSON_APPEND_DOCUMENT_BEGIN(setSortedStage, "t_sorted", &setSortedDoc);
	BSON_APPEND_DOCUMENT_BEGIN(&setSortedDoc, "$sortArray", &sortArrayDoc);
	BSON_APPEND_UTF8(&sortArrayDoc, "input", "$t_paths");
	BSON_APPEND_DOCUMENT_BEGIN(&sortArrayDoc, "$sortArray", &sortByDoc);
	BSON_APPEND_INT32(&sortByDoc, "depth", 1);
	bson_append_document_end(&sortArrayDoc, &sortByDoc);
	bson_append_document_end(&setSortedDoc, &sortArrayDoc);
	bson_append_document_end(setSortedStage, &setSortedDoc);
	pipeline.appendStageEnd(setSortedStage);

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

	// project new variable groundings
	setTripleVariables(pipeline, lookupData);
	// remove next field again: { $unset: "next" }
	pipeline.unset("next");
}

void mongo::lookupTriple(Pipeline &pipeline, const TripleStore &tripleStore, const TripleLookupData &lookupData) {
	semweb::PropertyPtr definedProperty;
	if (lookupData.expr->propertyTerm() && lookupData.expr->propertyTerm()->termType() == TermType::ATOMIC) {
		auto propertyTerm = std::static_pointer_cast<Atomic>(lookupData.expr->propertyTerm());
		definedProperty = tripleStore.vocabulary->getDefinedProperty(propertyTerm->stringForm());
	}

	bool b_isTransitiveProperty = (definedProperty && definedProperty->hasFlag(
			semweb::PropertyFlag::TRANSITIVE_PROPERTY));
	if (b_isTransitiveProperty)
		transitiveLookup(pipeline, tripleStore, lookupData, definedProperty);
	else
		nonTransitiveLookup(pipeline, tripleStore, lookupData, definedProperty);
}
