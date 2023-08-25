//
// Created by daniel on 03.04.23.
//

#include <sstream>
#include "knowrob/mongodb/aggregation/triples.h"
#include "knowrob/mongodb/aggregation/terms.h"
#include "knowrob/terms/Constant.h"
#include "knowrob/Logger.h"
#include "knowrob/terms/ListTerm.h"

#define MONGO_OPERATOR_LTE  "$lte"
#define MONGO_OPERATOR_GTE  "$gte"
#define MONGO_OPERATOR_LT   "$lt"
#define MONGO_OPERATOR_GT   "$gt"

using namespace knowrob;
using namespace knowrob::mongo;

static inline void matchSinceBeforeUntil(aggregation::Pipeline &pipeline)
{
    // { $match: { $expr: { $lte: ["$v_scope.time.since", "$v_scope.time.until"] } } }
    bson_t exprDoc, ltDoc;
    auto matchStage = pipeline.appendStageBegin("$match");
    BSON_APPEND_DOCUMENT_BEGIN(matchStage, "$expr", &exprDoc);
    BSON_APPEND_ARRAY_BEGIN(&exprDoc, "$lte", &ltDoc);
    BSON_APPEND_UTF8(&ltDoc, "0", "$v_scope.time.since");
    BSON_APPEND_UTF8(&ltDoc, "1", "$v_scope.time.until");
    bson_append_array_end(&exprDoc, &ltDoc);
    bson_append_document_end(matchStage, &exprDoc);
    pipeline.appendStageEnd(matchStage);
}

static inline void intersectTimeInterval(aggregation::Pipeline &pipeline,
                                         const char *newSinceValue,
                                         const char *newUntilValue)
{
    bson_t sinceDoc, untilDoc, maxArray, minArray;
    auto setStage = pipeline.appendStageBegin("$set"); {
        // "v_scope.time.since", { $max: ["$v_scope.time.since", "$next.scope.time.since"] }
        // note: $max [null,2] -> 2
        BSON_APPEND_DOCUMENT_BEGIN(setStage, "v_scope.time.since", &sinceDoc);
        BSON_APPEND_ARRAY_BEGIN(&sinceDoc, "$max", &maxArray);
        BSON_APPEND_UTF8(&maxArray, "0", "$v_scope.time.since");
        BSON_APPEND_UTF8(&maxArray, "1", newSinceValue);
        bson_append_array_end(&sinceDoc, &maxArray);
        bson_append_document_end(setStage, &sinceDoc);

        // "v_scope.time.until", { $min: ["$v_scope.time.until", "$next.scope.time.until"] }
        // note: $min [null,2] -> 2
        BSON_APPEND_DOCUMENT_BEGIN(setStage, "v_scope.time.until", &untilDoc);
        BSON_APPEND_ARRAY_BEGIN(&untilDoc, "$min", &minArray);
        BSON_APPEND_UTF8(&minArray, "0", "$v_scope.time.until");
        BSON_APPEND_UTF8(&minArray, "1", newUntilValue);
        bson_append_array_end(&untilDoc, &minArray);
        bson_append_document_end(setStage, &untilDoc);
    }
    pipeline.appendStageEnd(setStage);
}

static inline std::string getVariableKey(const std::string_view &varName)
{
    std::stringstream ss;
    ss << "v_VARS." << varName;
    return ss.str();
}

static inline void appendSetVariable(bson_t *doc,
                                     const std::string_view &varName,
                                     const std::string_view &newValue)
{
    const auto varKey = getVariableKey(varName)+".val";
    const auto varValue = std::string("$")+varKey;
    bson_t condDoc, condArray, notOperator, notArray;
    // Only conditionally apply new value in case it has not been grounded before.
    // varKey: {$cond: [ { $not: [ varValue ] }, newValue, varValue ]}
    BSON_APPEND_DOCUMENT_BEGIN(doc, varKey.c_str(), &condDoc);
    BSON_APPEND_ARRAY_BEGIN(&condDoc, "$cond", &condArray); {
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
                                       const aggregation::TripleLookupData &lookupData)
{
    const auto varKey = getVariableKey(varName)+".val";
    const auto varLetValue = std::string("$$")+varKey;
    const auto matchOperator = (fieldValue.back()=='*' ? "$in" : "$eq");
    bson_t exprDoc, orArr, notDoc, notArr, matchDoc, matchArr;

    BSON_APPEND_DOCUMENT_BEGIN(doc, "$expr", &exprDoc);

    if(lookupData.knownGroundedVariables.count(varName)>0) {
        // variable is known to have a grounding in the v_VARS field
        // {$expr: { matchOperator: [varLetValue, fieldValue] } }
        BSON_APPEND_ARRAY_BEGIN(&exprDoc, matchOperator, &matchArr);
        BSON_APPEND_UTF8(&matchArr, "0", varLetValue.c_str());
        BSON_APPEND_UTF8(&matchArr, "1", fieldValue.data());
        bson_append_array_end(&exprDoc, &matchArr);
    }
    else {
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
        aggregation::Pipeline &pipeline,
        const aggregation::TripleLookupData &lookupData)
{
    // TODO: consider storing the document id of the triple in which var is grounded.
    // TODO: consider storing the confidence value of triples
    //      - both would need a list as multiple predicates could back a grounding

    std::list<std::pair<const char*,Variable*>> varList;
    for(auto &it : {
            std::make_pair("$next.s",lookupData.expr->subjectTerm()),
            std::make_pair("$next.p",lookupData.expr->propertyTerm()),
            std::make_pair("$next.o",lookupData.expr->objectTerm())
    }) {
        if(it.second->type()!=TermType::VARIABLE) continue;
        auto var = (Variable*)it.second.get();
        // skip variables that were instantiated in previous steps
        if(lookupData.knownGroundedVariables.count(var->name())>0) continue;
        varList.emplace_back(it.first, var);
    }

    if(!varList.empty()) {
        auto setVariables = pipeline.appendStageBegin("$set");
        for(auto &it : varList) {
            appendSetVariable(setVariables, it.second->name(), it.first);
        }
        pipeline.appendStageEnd(setVariables);
    }
}

static inline const char* getOperatorString(knowrob::FramedRDFLiteral::OperatorType operatorType)
{
    switch(operatorType) {
        case FramedRDFLiteral::EQ:
            return nullptr;
        case FramedRDFLiteral::LEQ:
            return MONGO_OPERATOR_LTE;
        case FramedRDFLiteral::GEQ:
            return MONGO_OPERATOR_GTE;
        case FramedRDFLiteral::LT:
            return MONGO_OPERATOR_LT;
        case FramedRDFLiteral::GT:
            return MONGO_OPERATOR_GT;
    }
    return nullptr;
}

void aggregation::appendGraphSelector(bson_t *selectorDoc, const FramedRDFLiteral &tripleExpression)
{
    auto gt = tripleExpression.graphTerm();
    if(!gt) return;

    if(gt->type() == TermType::STRING) {
        auto graphString = (StringTerm*)gt.get();
        if(graphString->value()=="*" || graphString->value()=="user") {}
        else {
            aggregation::appendTermQuery(selectorDoc, "graph", gt);
        }
    }
    else if(gt->type() == TermType::LIST) {
        // TODO: support graph hierarchy lookups, i.e. include all solutions of imported graphs.
        //       imported graphs can be obtained from ImportHierarchy.
        //       then the graph term can be encoded as ListTerm of all imports and handled here.
        //       where should this list be constructed? it should still be possible to query
        //       for records of a particular graph
        KB_WARN("graph term {} has unexpected type", *gt);
    }
    else {
        KB_WARN("graph term {} has unexpected type", *gt);
    }
}

void aggregation::appendEpistemicSelector(bson_t *selectorDoc, const FramedRDFLiteral &tripleExpression)
{
    static const bool allowConfidenceNullValues = true;
    static auto zero = std::make_shared<Integer32Term>(0);
    auto ct = tripleExpression.confidenceTerm();
    auto at = tripleExpression.agentTerm();
    auto op = tripleExpression.modalityFrame().epistemicOperator();

    // enforce that uncertain=false in case knowledge modality is selected in query
    if(op && op->isModalNecessity()) {
        // note: null value of "uncertain" field is seen as value "false"
        aggregation::appendTermQuery(
                    selectorDoc,
                    "uncertain",
                    zero,
                    nullptr,
                    true);
    }

    if(at) {
        if(at->type() == TermType::STRING) {
            aggregation::appendTermQuery(
                selectorDoc,
                "agent",
                at,
                nullptr,
                false);
        }
        else {
            KB_WARN("agent term {} has unexpected type", at);
        }
    }
    else {
        // make sure agent field is undefined: { agent: { $exists: false } }
        // note: null value of agent field is seen as "self", i.e. the agent running the knowledge base
        bson_t agentDoc;
        BSON_APPEND_DOCUMENT_BEGIN(selectorDoc, "agent", &agentDoc);
        BSON_APPEND_BOOL(&agentDoc, "$exists", false);
        bson_append_document_end(selectorDoc, &agentDoc);
    }

    if(ct) {
        if(ct->type() == TermType::DOUBLE) {
            // note: null value of confidence is seen as larger than the requested threshold
            aggregation::appendTermQuery(
                    selectorDoc,
                    "confidence",
                    ct,
                    MONGO_OPERATOR_GTE,
                    true);
        }
        else {
            KB_WARN("confidence term {} has unexpected type", *ct);
        }
    }
}

void aggregation::appendTimeSelector(bson_t *selectorDoc, const FramedRDFLiteral &tripleExpression)
{
    static const bool allowNullValues = true;
    static auto b_occasional = std::make_shared<Integer32Term>(static_cast<int32_t>(true));
    static auto b_always = std::make_shared<Integer32Term>(static_cast<int32_t>(false));
    auto bt = tripleExpression.beginTerm();
    auto et = tripleExpression.endTerm();

    // matching must be done depending on temporal operator:
    // - H: bt >= since_H && et <= until_H
    // - P: et >= since_H && bt <= until_H
    // - TODO: there is another case for operator P: bt <= since_P && et >= until_P
    auto &mf = tripleExpression.modalityFrame();
    if(mf.isAboutSomePast()) {
        // just swap bt/et (see above comment)
        auto swap = bt;
        bt = et;
        et = swap;
    }
    // ensure that input document has *H* operator.
    // null is also ok. in particular exclude documents with *P* operator here.
    aggregation::appendTermQuery(
        selectorDoc,
        "occasional",
        b_always,
        nullptr,
        allowNullValues);

    if(bt) {
        if(bt->type() == TermType::DOUBLE) {
            aggregation::appendTermQuery(
                    selectorDoc,
                    "scope.time.since",
                    bt,
                    MONGO_OPERATOR_LTE,
                    allowNullValues);
        }
        else {
            KB_WARN("begin term {} has unexpected type", *bt);
        }
    }
    if(et) {
        if(et->type() == TermType::DOUBLE) {
            aggregation::appendTermQuery(
                    selectorDoc,
                    "scope.time.until",
                    et,
                    MONGO_OPERATOR_GTE,
                    allowNullValues);
        }
        else {
            KB_WARN("end term {} has unexpected type", *et);
        }
    }
}

void aggregation::appendTripleSelector(
            bson_t *selectorDoc,
            const FramedRDFLiteral &tripleExpression,
            bool b_isTaxonomicProperty)
{
    // "s"" field
    aggregation::appendTermQuery(selectorDoc,
        "s", tripleExpression.subjectTerm());
    // "p" field
    aggregation::appendTermQuery(selectorDoc,
            (b_isTaxonomicProperty ? "p" : "p*"),
            tripleExpression.propertyTerm());
    // "o" field
    const char* objectOperator = getOperatorString(tripleExpression.objectOperator());
    aggregation::appendTermQuery(selectorDoc,
            (b_isTaxonomicProperty ? "o*" : "o"),
            tripleExpression.objectTerm(),
            objectOperator);
    // "g" field
    appendGraphSelector(selectorDoc, tripleExpression);
    // epistemic fields
    appendEpistemicSelector(selectorDoc, tripleExpression);
    // temporal fields
    appendTimeSelector(selectorDoc, tripleExpression);
}

static inline void lookupTriple_nontransitive_(
        aggregation::Pipeline &pipeline,
        const std::string_view &collection,
        const std::shared_ptr<semweb::Vocabulary> &vocabulary,
        const aggregation::TripleLookupData &lookupData,
        const semweb::PropertyPtr &definedProperty)
{
    bool b_isTaxonomicProperty = (definedProperty &&
                vocabulary->isTaxonomicProperty(definedProperty->iri()));
    bool b_isReflexiveProperty = (definedProperty &&
                definedProperty->hasFlag(semweb::PropertyFlag::REFLEXIVE_PROPERTY));
    char arrIndexStr[16];
    const char *arrIndexKey;
    uint32_t arrIndex;

    bool b_skipInputGroundings=false;
    if(lookupData.expr->isGround()) {
        // the triple expression has no variables
        b_skipInputGroundings = true;
    }
    else if(!lookupData.mayHasMoreGroundings) {
        // all possible previous instantiations are known and stored
        // in lookupData.knownGroundedVariables.
        b_skipInputGroundings = true;
        for(auto &exprTerm : {
                lookupData.expr->subjectTerm(),
                lookupData.expr->propertyTerm(),
                lookupData.expr->objectTerm()
        }) {
            if(exprTerm->type()!=TermType::VARIABLE) continue;
            auto var = (Variable*)exprTerm.get();
            // skip if this variable cannot have a runtime grounding
            if(lookupData.knownGroundedVariables.count(var->name())>0) {
                // the expression contains a variable that is known to have received
                // a grounding before
                b_skipInputGroundings = false;
                break;
            }
        }
    }

    // filter out documents that do not match the triple pattern.
  	// this is done using $match or $lookup operators.
  	// first lookup matching documents and store them in the 'next' field
    bson_t lookupArray, letDoc;
    auto lookupStage = pipeline.appendStageBegin("$lookup");
    BSON_APPEND_UTF8(lookupStage, "from", collection.data());
    BSON_APPEND_UTF8(lookupStage, "as", "next");
    if(!b_skipInputGroundings) {
        // pass "v_VARS" field to lookup pipeline
        BSON_APPEND_DOCUMENT_BEGIN(lookupStage, "let", &letDoc);
        BSON_APPEND_UTF8(&letDoc, "v_VARS", "$v_VARS");
        bson_append_document_end(lookupStage, &letDoc);
    }
    BSON_APPEND_ARRAY_BEGIN(lookupStage, "pipeline", &lookupArray); {
        aggregation::Pipeline lookupPipeline(&lookupArray);

        auto matchStage = lookupPipeline.appendStageBegin("$match");
        if(b_skipInputGroundings) {
            aggregation::appendTripleSelector(matchStage,
                                              *lookupData.expr,
                                              b_isTaxonomicProperty);
        }
        else {
            // need to match with potential groundings of variables from previous steps
            // these are stored in the "v_VARS" field.
            bson_t andArray, tripleDoc, variablesDoc;
            BSON_APPEND_ARRAY_BEGIN(matchStage, "$and", &andArray); {
                BSON_APPEND_DOCUMENT_BEGIN(&andArray, "0", &tripleDoc);
                aggregation::appendTripleSelector(&tripleDoc, *lookupData.expr, b_isTaxonomicProperty);
                bson_append_document_end(&andArray, &tripleDoc);

                // match triple values with previously grounded variables
                arrIndex = 1;
                for(auto &it : {
                        std::make_pair("$s",lookupData.expr->subjectTerm()),
                        std::make_pair(b_isTaxonomicProperty ? "$p" : "$p*",lookupData.expr->propertyTerm()),
                        std::make_pair(b_isTaxonomicProperty ? "$o*" : "$o",lookupData.expr->objectTerm())
                }) {
                    if(it.second->type()!=TermType::VARIABLE) continue;
                    auto var = (Variable*)it.second.get();
                    // skip if this variable cannot have a runtime grounding
                    if(!lookupData.mayHasMoreGroundings &&
                       lookupData.knownGroundedVariables.count(var->name())==0) continue;

                    bson_uint32_to_string (arrIndex++,
                        &arrIndexKey, arrIndexStr, sizeof arrIndexStr);
                    BSON_APPEND_DOCUMENT_BEGIN(&andArray, arrIndexKey, &variablesDoc);
                    appendMatchVariable(&variablesDoc,
                                        it.first,
                                        var->name(),
                                        lookupData);
                    bson_append_document_end(&andArray, &variablesDoc);
                }
            }
            bson_append_array_end(matchStage, &andArray);
        }
        lookupPipeline.appendStageEnd(matchStage);
        // { $limit: maxNumOfTriples }
        if(lookupData.maxNumOfTriples>0) lookupPipeline.limit(lookupData.maxNumOfTriples);
    }
    bson_append_array_end(lookupStage, &lookupArray);
    pipeline.appendStageEnd(lookupStage);

    // TODO: add additional results if P is a reflexive property
    if(b_isReflexiveProperty) {
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

    // at this point the 'next' field holds an array of matching documents that is unwinded next.
    pipeline.unwind("$next");
    // compute the intersection of time interval so far with time interval of next triple.
    // note that the operations works fine in case the time interval is undefined.
    // TODO: below time interval computation is only ok assuming the statements are not "occasional"
    intersectTimeInterval(pipeline,
                          "$next.scope.time.since",
                          "$next.scope.time.until");
   	// then verify that the scope is non-empty.
   	matchSinceBeforeUntil(pipeline);
    // TODO: maintain "uncertain" field:
    //      - set(v_scope.uncertain = ($v_scope.uncertain || next.uncertain))
    // project new variable groundings
    setTripleVariables(pipeline, lookupData);
    // remove next field again: { $unset: "next" }
    pipeline.unset("next");
}

static inline void lookupTriple_transitive_(
        aggregation::Pipeline &pipeline,
        const std::string_view &collection,
        const std::shared_ptr<semweb::Vocabulary> &vocabulary,
        const aggregation::TripleLookupData &lookupData,
        const semweb::PropertyPtr &definedProperty)
{
    bool b_isReflexiveProperty = (definedProperty &&
                definedProperty->hasFlag(semweb::PropertyFlag::REFLEXIVE_PROPERTY));
    bool b_isTaxonomicProperty = (definedProperty &&
                vocabulary->isTaxonomicProperty(definedProperty->iri()));
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
    BSON_APPEND_UTF8(lookupStage, "from", collection.data());
    if(startTerm->type() == TermType::STRING) {
        auto startString = (StringTerm*)startTerm.get();
        BSON_APPEND_UTF8(lookupStage, "startWith", startString->value().c_str());
    }
    else if(startTerm->type() == TermType::VARIABLE) {
        auto startVariable = (Variable*)startTerm.get();
        auto startValue = std::string("$") + getVariableKey(startVariable->name()) + ".val";
        BSON_APPEND_UTF8(lookupStage, "startWith", startValue.c_str());
    }
    else if(startTerm->type() == TermType::LIST) {
        // TODO: support array values as start for graph lookup queries
        KB_WARN("Ignoring array {} for graph lookup, not supported yet.", *startTerm);
    }
    else {
        KB_WARN("Ignoring term {} with invalid type for graph lookup.", *startTerm);
    }
    BSON_APPEND_UTF8(lookupStage, "connectToField",     b_startWithSubject ? "s" : "o");
    BSON_APPEND_UTF8(lookupStage, "connectFromField",   b_startWithSubject ? "p" : "s");
    BSON_APPEND_UTF8(lookupStage, "as", "t_paths");
    BSON_APPEND_UTF8(lookupStage, "depthField", "depth");
    /* { restrictSearchWithMatch: { p*: Query_p, ... }" */
    BSON_APPEND_DOCUMENT_BEGIN(lookupStage, "restrictSearchWithMatch", &restrictSearchDoc); {
        aggregation::appendTermQuery(&restrictSearchDoc,
                                     (b_isTaxonomicProperty ? "p" : "p*"),
                                     lookupData.expr->propertyTerm());
        aggregation::appendGraphSelector(&restrictSearchDoc, *lookupData.expr);
        aggregation::appendEpistemicSelector(&restrictSearchDoc,*lookupData.expr);
        aggregation::appendTimeSelector(&restrictSearchDoc, *lookupData.expr);
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
    BSON_APPEND_ARRAY_BEGIN(orderingStage, "pipeline", &orderingArray); {
        aggregation::Pipeline orderingPipeline(&orderingArray);
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

    if(b_isReflexiveProperty) {
        // TODO: handle reflexivity in triple graph lookup
        // reflexivity(StartValue, Ctx, Step)
    }

    // iterate over results: { $unwind: "$next" }
    pipeline.unwind("$next");

    // graph lookup uses "s" or "o" as start for recursive lookup but ignores the other.
    // thus a matching must be performed for the results.
    if(endTerm->type() != TermType::VARIABLE) {
        // FIXME: must add another match case for endTerm being a runtime grounded variable.
        bson_t  matchEndVal;
        auto matchEnd = pipeline.appendStageBegin("$match");
        BSON_APPEND_DOCUMENT_BEGIN(matchEnd, "next.o", &matchEndVal);
        aggregation::appendTermQuery(&restrictSearchDoc,
                                     (b_startWithSubject ? "next.o" : "next.s"),
                                     endTerm);
        bson_append_document_end(matchEnd, &matchEndVal);
        pipeline.appendStageEnd(matchEnd);
    }

    // compute the intersection of time interval so far with time interval of next triple.
    // note that the operations work fine in case the time interval is undefined.
    // FIXME: intersection need to be performed over all transitions in graph lookup
    intersectTimeInterval(pipeline,
                          "$next.scope.time.since",
                          "$next.scope.time.until");
   	// then verify that the scope is non-empty.
   	matchSinceBeforeUntil(pipeline);
    // project new variable groundings
    setTripleVariables(pipeline, lookupData);
    // remove next field again: { $unset: "next" }
    pipeline.unset("next");
}

void aggregation::lookupTriple(
        aggregation::Pipeline &pipeline,
        const std::string_view &collection,
        const std::shared_ptr<semweb::Vocabulary> &vocabulary,
        const TripleLookupData &lookupData)
{
    // lookup defined properties, there are some conditions in lookup on property
    // semantics.
    semweb::PropertyPtr definedProperty;
    if(lookupData.expr->propertyTerm()->type()==TermType::STRING) {
        auto propertyTerm = (StringTerm*)lookupData.expr->propertyTerm().get();
        definedProperty = vocabulary->getDefinedProperty(propertyTerm->value());
    }

    bool b_isTransitiveProperty = (definedProperty && definedProperty->hasFlag(
            semweb::PropertyFlag::TRANSITIVE_PROPERTY));
    if(b_isTransitiveProperty || lookupData.forceTransitiveLookup)
        lookupTriple_transitive_(pipeline, collection, vocabulary, lookupData, definedProperty);
    else
        lookupTriple_nontransitive_(pipeline, collection, vocabulary, lookupData, definedProperty);
}

void aggregation::lookupTriplePaths(
        aggregation::Pipeline &pipeline,
        const std::string_view &collection,
        const std::shared_ptr<semweb::Vocabulary> &vocabulary,
        const std::vector<FramedRDFLiteralPtr> &tripleExpressions)
{
    std::set<std::string_view> varsSoFar;

    // FIXME: need to handle negative literals here
    for(auto &expr : tripleExpressions) {
        // append lookup stages to pipeline
        aggregation::TripleLookupData lookupData(expr.get());

        // indicate that all previous groundings of variables are known
        lookupData.mayHasMoreGroundings = false;
        lookupData.knownGroundedVariables = varsSoFar;
        // remember variables in tripleExpression, they have a grounding in next step
        for(auto &exprTerm : {
                expr->subjectTerm(), expr->propertyTerm(), expr->objectTerm() }) {
            if(exprTerm->type()==TermType::VARIABLE)
                varsSoFar.insert(((Variable*)exprTerm.get())->name());
        }

        aggregation::lookupTriple(pipeline,
                                  collection,
                                  vocabulary,
                                  lookupData);
    }
}
