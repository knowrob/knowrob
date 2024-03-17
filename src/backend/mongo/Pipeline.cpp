/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "string"
#include "knowrob/backend/mongo/Pipeline.h"
#include "knowrob/Logger.h"
#include "knowrob/URI.h"
#include "knowrob/backend/mongo/aggregation/triples.h"
#include "knowrob/triples/GraphPattern.h"
#include "knowrob/triples/GraphSequence.h"
#include "knowrob/triples/GraphBuiltin.h"
#include "knowrob/backend/mongo/MongoTerm.h"

using namespace knowrob::mongo;

Pipeline::Pipeline(bson_t *arrayDocument)
		: arrayDocument_(arrayDocument),
		  numStages_(0),
		  lastStage_(nullptr),
		  lastOperator_(nullptr) {
}

bson_t *Pipeline::appendStageBegin() {
	auto arrayKey = std::to_string(numStages_++);
	bson_wrapper &stage = stages_.emplace_back();
	BSON_APPEND_DOCUMENT_BEGIN(arrayDocument_, arrayKey.c_str(), &stage.bson);
	lastStage_ = &stage.bson;
	return &stage.bson;
}

bson_t *Pipeline::appendStageBegin(const char *stageOperatorString) {
	auto arrayKey = std::to_string(numStages_++);
	bson_wrapper &stage = stages_.emplace_back();
	bson_wrapper &stageOperator = stageOperators_.emplace_back();
	BSON_APPEND_DOCUMENT_BEGIN(arrayDocument_, arrayKey.c_str(), &stage.bson);
	BSON_APPEND_DOCUMENT_BEGIN(&stage.bson, stageOperatorString, &stageOperator.bson);
	lastOperator_ = &stageOperator.bson;
	lastStage_ = &stage.bson;
	return &stageOperator.bson;
}

void Pipeline::appendStageEnd(bson_t *stage) {
	if (lastOperator_ == stage) {
		bson_append_document_end(lastStage_, lastOperator_);
		bson_append_document_end(arrayDocument_, lastStage_);
	} else {
		bson_append_document_end(arrayDocument_, stage);
	}
}

void Pipeline::append(const knowrob::FramedTriplePattern &query, const TripleStore &tripleStore) {
	// append lookup stages to pipeline
	aggregation::TripleLookupData lookupData(&query);
	// indicate that no variables in tripleExpression may have been instantiated
	// by a previous step to allow for some optimizations.
	lookupData.mayHasMoreGroundings = false;
	aggregation::lookupTriple(*this, tripleStore, lookupData);
}

void Pipeline::append(const knowrob::GraphTerm &query, const TripleStore &tripleStore) {
	std::set<std::string_view> groundedVariables;
	appendTerm_recursive(query, tripleStore, groundedVariables);
}

void Pipeline::appendTerm_recursive(const knowrob::GraphTerm &query, // NOLINT
									const TripleStore &tripleStore,
									std::set<std::string_view> &groundedVariables) {
	switch (query.termType()) {
		case knowrob::GraphTermType::Pattern: {
			auto &expr = ((const GraphPattern &) query).value();
			aggregation::TripleLookupData lookupData(expr.get());
			// indicate that all previous groundings of variables are known
			lookupData.mayHasMoreGroundings = false;
			lookupData.knownGroundedVariables = groundedVariables;
			// remember variables in tripleExpression, they have a grounding in next step
			for (auto &var: expr->getVariables()) {
				groundedVariables.insert(var->name());
			}
			aggregation::lookupTriple(*this, tripleStore, lookupData);
			break;
		}
		case knowrob::GraphTermType::Builtin:
			appendBuiltin((const knowrob::GraphBuiltin &) query);
			break;
		case knowrob::GraphTermType::Sequence:
			for (auto &elem: ((const knowrob::GraphSequence &) query).terms()) {
				appendTerm_recursive(*elem, tripleStore, groundedVariables);
			}
			break;
		case knowrob::GraphTermType::Union:
			appendUnion((const knowrob::GraphUnion &) query);
			break;
	}
}

void Pipeline::appendUnion(const knowrob::GraphUnion &builtin) {
	// TODO: Support Union operation in MongoDB knowledge graph.
	//       afaik, this can only be achieved through multiple $lookup stages
	//       then use $concatArrays and $unwind. $union does not work with variables!
	// TODO: let variables need to be accessed with $$, so passing v_VARS into pipeline
	//       is not enough to make it work. mainly affects the triple lookup code.
	/*
aggregate_disjunction(FindallStages, StepVars, Pipeline, StepVars) :-
	% get a list of list variable keys
	findall(string(X),
		member([_,X,_],FindallStages),
		VarKeys),
	% prepend "$" for accessing values
	maplist([string(In),string(Out)]>>
		atom_concat('$',In,Out),
		VarKeys, VarValues),
	%
	findall(QueryStage,
		% first, compute array of results for each facet
		(	member([QueryStage,_,_], FindallStages)
		% second, concatenate the results
		;	QueryStage=['$set', ['next', ['$concatArrays', array(VarValues)]]]
		% third, delete unneeded array
		;	QueryStage=['$unset', array(VarKeys)]
		% unwind all solutions from disjunction
		;	QueryStage=['$unwind', string('$next')]
		% finally project the result of a disjunction goal
		;	mongolog:set_next_vars(StepVars, QueryStage)
		% and unset the next field
		;	QueryStage=['$unset', string('next')]
		),
		Pipeline
	).
	 */
	KB_WARN("Union not supported in pipeline yet");
}

void Pipeline::appendBuiltin(const knowrob::GraphBuiltin &builtin) {
	switch (builtin.builtinType()) {
		case knowrob::GraphBuiltinType::Bind:
			bindValue(builtin);
			break;
		case knowrob::GraphBuiltinType::Max:
			setAccumulated(builtin, "$max");
			break;
		case knowrob::GraphBuiltinType::Min:
			setAccumulated(builtin, "$min");
			break;
		case knowrob::GraphBuiltinType::LessOrEqual:
			matchBinary(builtin, "$lte");
			break;
		case knowrob::GraphBuiltinType::Less:
			matchBinary(builtin, "$lt");
			break;
		case knowrob::GraphBuiltinType::Greater:
			matchBinary(builtin, "$gt");
			break;
		case knowrob::GraphBuiltinType::GreaterOrEqual:
			matchBinary(builtin, "$gte");
			break;
		case knowrob::GraphBuiltinType::Equal:
			matchBinary(builtin, "$eq");
			break;
	}
}

void Pipeline::bindValue(const knowrob::GraphBuiltin &builtin) {
	// e.g. `{ $set: { "begin": "$next.begin" } }`
	if (!builtin.bindVar()) {
		KB_ERROR("No variable to bind in $min/$max operation");
		return;
	}
	if (builtin.arguments().size() != 1) {
		KB_ERROR("Bind operation requires one argument");
		return;
	}
	static const std::string varPrefix = "v_VARS.";
	static const std::string varSuffix = ".val";
	auto setStage = appendStageBegin("$set");
	auto varKey = varPrefix + std::string(builtin.bindVar()->name()) + varSuffix;
	MongoTerm::appendWithVars(setStage, varKey.c_str(), builtin.arguments()[0]);
	appendStageEnd(setStage);
}

void Pipeline::setAccumulated(const knowrob::GraphBuiltin &builtin, std::string_view predicate) {
	// e.g. `{ $set:  "begin", { $max: ["$begin", "$next.begin"] } }`
	// NOTE: `$min [null,2]` -> 2 and `$max [null,2]` -> 2
	if (!builtin.bindVar()) {
		KB_ERROR("No variable to bind in $min/$max operation");
		return;
	}
	static const std::string varPrefix = "v_VARS.";
	static const std::string varSuffix = ".val";
	bson_t accumulatedDoc, inputArray;
	auto setStage = appendStageBegin("$set");
	auto varKey = varPrefix + std::string(builtin.bindVar()->name()) + varSuffix;
	BSON_APPEND_DOCUMENT_BEGIN(setStage, varKey.c_str(), &accumulatedDoc);
	BSON_APPEND_ARRAY_BEGIN(&accumulatedDoc, predicate.data(), &inputArray);
	for (uint32_t i = 0; i < builtin.arguments().size(); i++) {
		MongoTerm::appendWithVars(&inputArray, std::to_string(i).c_str(), builtin.arguments()[i]);
	}
	bson_append_array_end(&accumulatedDoc, &inputArray);
	bson_append_document_end(setStage, &accumulatedDoc);
	appendStageEnd(setStage);
}

void Pipeline::matchBinary(const knowrob::GraphBuiltin &builtin, std::string_view predicate) {
	// e.g.: `{ $match: { $expr: { $lte: ["$v_scope.begin", "$v_scope.end"] } } }`
	if (builtin.arguments().size() != 2) {
		KB_ERROR("Binary operation requires two arguments");
		return;
	}
	bson_t exprDoc, ltDoc;
	auto matchStage = appendStageBegin("$match");
	BSON_APPEND_DOCUMENT_BEGIN(matchStage, "$expr", &exprDoc);
	BSON_APPEND_ARRAY_BEGIN(&exprDoc, predicate.data(), &ltDoc);
	MongoTerm::appendWithVars(&ltDoc, "0", builtin.arguments()[0]);
	MongoTerm::appendWithVars(&ltDoc, "1", builtin.arguments()[1]);
	bson_append_array_end(&exprDoc, &ltDoc);
	bson_append_document_end(matchStage, &exprDoc);
	appendStageEnd(matchStage);
}

void Pipeline::limit(uint32_t maxDocuments) {
	auto unwindStage = appendStageBegin();
	BSON_APPEND_INT32(unwindStage, "$limit", maxDocuments);
	appendStageEnd(unwindStage);
}

void Pipeline::unwind(const std::string_view &field) {
	auto unwindStage = appendStageBegin();
	BSON_APPEND_UTF8(unwindStage, "$unwind", field.data());
	appendStageEnd(unwindStage);
}

void Pipeline::unset(const std::string_view &field) {
	auto unwindStage = appendStageBegin();
	BSON_APPEND_UTF8(unwindStage, "$unset", field.data());
	appendStageEnd(unwindStage);
}

void Pipeline::replaceRoot(const std::string_view &newRootField) {
	auto unwindStage = appendStageBegin("$replaceRoot");
	BSON_APPEND_UTF8(unwindStage, "newRoot", newRootField.data());
	appendStageEnd(unwindStage);
}

void Pipeline::sortAscending(const std::string_view &field) {
	auto sortStage = appendStageBegin("$sort");
	BSON_APPEND_INT32(sortStage, field.data(), 1);
	appendStageEnd(sortStage);
}

void Pipeline::sortDescending(const std::string_view &field) {
	auto sortStage = appendStageBegin("$sort");
	BSON_APPEND_INT32(sortStage, field.data(), -1);
	appendStageEnd(sortStage);
}

void Pipeline::merge(const std::string_view &collection) {
	auto unwindStage = appendStageBegin("$merge");
	BSON_APPEND_UTF8(unwindStage, "into", collection.data());
	BSON_APPEND_UTF8(unwindStage, "on", "_id");
	BSON_APPEND_UTF8(unwindStage, "whenMatched", "merge");
	appendStageEnd(unwindStage);
}

void Pipeline::project(const std::string_view &field) {
	auto projectStage = appendStageBegin("$project");
	BSON_APPEND_INT32(projectStage, field.data(), 1);
	appendStageEnd(projectStage);
}

void Pipeline::project(const std::vector<std::string_view> &fields) {
	auto projectStage = appendStageBegin("$project");
	for (auto field: fields) {
		BSON_APPEND_INT32(projectStage, field.data(), 1);
	}
	appendStageEnd(projectStage);
}

void Pipeline::setUnion(const std::string_view &field, const std::vector<std::string_view> &sets) {
	bson_t unionOperator, unionArray;
	uint32_t numElements = 0;
	auto setStage = appendStageBegin("$set");
	BSON_APPEND_DOCUMENT_BEGIN(setStage, field.data(), &unionOperator);
	{
		BSON_APPEND_ARRAY_BEGIN(&unionOperator, "$setUnion", &unionArray);
		{
			for (auto setString: sets) {
				auto arrayKey = std::to_string(numElements++);
				BSON_APPEND_UTF8(&unionArray, arrayKey.c_str(), setString.data());
			}
		}
		bson_append_array_end(&unionOperator, &unionArray);
	}
	bson_append_document_end(setStage, &unionOperator);
	appendStageEnd(setStage);
}

void Pipeline::addToArray(const std::string_view &key, const std::string_view &arrayKey,
						  const std::string_view &elementKey) {
	bson_t concatOperator, concatArray, concatArray1;
	auto setStage1 = appendStageBegin("$set");
	BSON_APPEND_DOCUMENT_BEGIN(setStage1, key.data(), &concatOperator);
	{
		BSON_APPEND_ARRAY_BEGIN(&concatOperator, "$concatArrays", &concatArray);
		{
			BSON_APPEND_UTF8(&concatArray, "0", arrayKey.data());
			BSON_APPEND_ARRAY_BEGIN(&concatArray, "1", &concatArray1);
			{
				BSON_APPEND_UTF8(&concatArray1, "0", elementKey.data());
			}
			bson_append_array_end(&concatArray, &concatArray1);
		}
		bson_append_array_end(&concatOperator, &concatArray);
	}
	bson_append_document_end(setStage1, &concatOperator);
	appendStageEnd(setStage1);
}

void replaceAll(std::string &str, const std::string &from, const std::string &to) {
	size_t startPos = 0;
	while ((startPos = str.find(from, startPos)) != std::string::npos) {
		str.replace(startPos, from.length(), to);
		startPos += to.length(); // Handles case where 'to' is a substring of 'from'
	}
}

bson_t *Pipeline::loadFromJSON(std::string_view filename, const std::map<std::string, std::string> &parameters) {
	auto resolved = URI::resolve(filename);
	// Load JSON file
	boost::property_tree::ptree pt;
	boost::property_tree::read_json(resolved, pt);

	// Convert JSON to string
	std::stringstream ss;
	boost::property_tree::write_json(ss, pt);

	// Replace placeholders with actual values
	std::string pipeline = ss.str();
	for (const auto &param: parameters) {
		replaceAll(pipeline, "${" + param.first + "}", param.second);
	}

	// Convert JSON to BSON
	bson_error_t error;
	bson_t *bson = bson_new_from_json((const uint8_t *) pipeline.c_str(), pipeline.size(), &error);

	if (!bson) {
		KB_ERROR("Error loading pipeline: {}", error.message);
		return nullptr;
	}

	return bson;
}
