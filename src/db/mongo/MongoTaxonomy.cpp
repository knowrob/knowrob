/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <mongoc.h>
#include <set>
#include "knowrob/db/mongo/MongoTaxonomy.h"
#include "knowrob/db/mongo/Pipeline.h"
#include "knowrob/semweb/rdfs.h"

using namespace knowrob;
using namespace knowrob::mongo;
using namespace knowrob::semweb;

MongoTaxonomy::MongoTaxonomy(
		const std::shared_ptr<mongo::Collection> &tripleCollection,
		const std::shared_ptr<mongo::Collection> &oneCollection)
		: tripleCollection_(tripleCollection), oneCollection_(oneCollection) {
}

void MongoTaxonomy::update(
		const std::vector<StringPair> &subClassAssertions,
		const std::vector<StringPair> &subPropertyAssertions) {
	// below performs the server-side data transformation for updating hierarchy relations
	// such as rdf::type.
	// However, there are many steps for large ontologies so this might consume some time.
	// TODO: list of parents could be supplied as a constant in aggregation queries below.
	//       currently parents are computed in the query, maybe it would be a bit faster using a constant
	//       baked into the query.

	bson_t pipelineDoc = BSON_INITIALIZER;

	// update class hierarchy.
	// unfortunately must be done step-by-step as it is undefined yet in mongo
	// if it's possible to access $merge results in following pipeline iterations via e.g. $lookup.
	for (auto &assertion: subClassAssertions) {
		bson_reinit(&pipelineDoc);

		bson_t pipelineArray;
		BSON_APPEND_ARRAY_BEGIN(&pipelineDoc, "pipeline", &pipelineArray);
		Pipeline pipeline(&pipelineArray);
		updateHierarchyO(pipeline,
						 tripleCollection_->name(),
						 rdfs::subClassOf->stringForm(),
						 assertion.first,
						 assertion.second);
		bson_append_array_end(&pipelineDoc, &pipelineArray);

		oneCollection_->evalAggregation(&pipelineDoc);
	}

	// update property hierarchy.
	// unfortunately must be done step-by-step as it is undefined yet in mongo
	// if it's possible to access $merge results in following pipeline iterations via e.g. $lookup.
	std::set<std::string_view> visited;
	for (auto &assertion: subPropertyAssertions) {
		visited.insert(assertion.first);
		bson_reinit(&pipelineDoc);

		bson_t pipelineArray;
		BSON_APPEND_ARRAY_BEGIN(&pipelineDoc, "pipeline", &pipelineArray);
		Pipeline pipeline(&pipelineArray);
		updateHierarchyO(pipeline,
						 tripleCollection_->name(),
						 rdfs::subPropertyOf->stringForm(),
						 assertion.first,
						 assertion.second);
		bson_append_array_end(&pipelineDoc, &pipelineArray);

		oneCollection_->evalAggregation(&pipelineDoc);
	}

	// update property assertions
	// TODO: below steps are independent, and could run in parallel.
	//       could bake an array of properties into pipeline,
	//       or rather use a bulk operation.
	for (auto &newProperty: visited) {
		bson_reinit(&pipelineDoc);

		bson_t pipelineArray;
		BSON_APPEND_ARRAY_BEGIN(&pipelineDoc, "pipeline", &pipelineArray);
		Pipeline pipeline(&pipelineArray);
		updateHierarchyP(pipeline,
						 tripleCollection_->name(),
						 rdfs::subPropertyOf->stringForm(),
						 newProperty);
		bson_append_array_end(&pipelineDoc, &pipelineArray);

		oneCollection_->evalAggregation(&pipelineDoc);
	}

	bson_destroy(&pipelineDoc);
}

void MongoTaxonomy::lookupParents(
		Pipeline &pipeline,
		const std::string_view &collection,
		const std::string_view &entity,
		const std::string_view &relation) {
	// lookup parent hierarchy.
	// e.g. for subClassOf these are all o* values of subClassOf documents of entity
	bson_t lookupArray;
	auto lookupStage = pipeline.appendStageBegin("$lookup");
	BSON_APPEND_UTF8(lookupStage, "from", collection.data());
	BSON_APPEND_UTF8(lookupStage, "as", "directParents");
	BSON_APPEND_ARRAY_BEGIN(lookupStage, "pipeline", &lookupArray);
	{
		Pipeline lookupPipeline(&lookupArray);
		// { $match: { s: $entity, p: $relation } }
		auto matchStage = lookupPipeline.appendStageBegin("$match");
		BSON_APPEND_UTF8(matchStage, "s", entity.data());
		BSON_APPEND_UTF8(matchStage, "p", relation.data());
		lookupPipeline.appendStageEnd(matchStage);
		// { $project: { "o*": 1 } }
		lookupPipeline.project("o*");
		// { $unwind: "$o*" }
		lookupPipeline.unwind("$o*");
	}
	bson_append_array_end(lookupStage, &lookupArray);
	pipeline.appendStageEnd(lookupStage);

	// convert "parents" field from list of documents to list of strings:
	// { $set { parents: { $map: { input: "$parents", "in": "$$this.o*" } } } }
	bson_t parentsDoc, mapDoc;
	auto setStage = pipeline.appendStageBegin("$set");
	BSON_APPEND_DOCUMENT_BEGIN(setStage, "directParents", &parentsDoc);
	{
		BSON_APPEND_DOCUMENT_BEGIN(&parentsDoc, "$map", &mapDoc);
		{
			BSON_APPEND_UTF8(&mapDoc, "input", "$directParents");
			BSON_APPEND_UTF8(&mapDoc, "in", "$$this.o*");
		}
		bson_append_document_end(&parentsDoc, &mapDoc);
	}
	bson_append_document_end(setStage, &parentsDoc);
	pipeline.appendStageEnd(setStage);
}

void MongoTaxonomy::updateHierarchyO(
		Pipeline &pipeline,
		const std::string_view &collection,
		const std::string_view &relation,
		const std::string_view &newChild,
		const std::string_view &newParent) {
	// lookup hierarchy into array field "parents"
	lookupParents(pipeline, collection, newParent, relation);
	// add newParent to parents array:
	// { $set: { parents: { $concatArrays: [ "$parents", [$newParent] ] } } }
	pipeline.addToArray("directParents", "$directParents", newParent);

	// lookup documents that include the child in the p* field
	bson_t lookupArray;
	auto lookupStage = pipeline.appendStageBegin("$lookup");
	BSON_APPEND_UTF8(lookupStage, "from", collection.data());
	BSON_APPEND_UTF8(lookupStage, "as", "doc");
	BSON_APPEND_ARRAY_BEGIN(lookupStage, "pipeline", &lookupArray);
	{
		Pipeline lookupPipeline(&lookupArray);
		// { $match: { "o*": $newChild } }
		auto matchStage = lookupPipeline.appendStageBegin("$match");
		BSON_APPEND_UTF8(matchStage, "o*", newChild.data());
		lookupPipeline.appendStageEnd(matchStage);
		// { $project: { "o*": 1 } }
		lookupPipeline.project("o*");
	}
	bson_append_array_end(lookupStage, &lookupArray);
	pipeline.appendStageEnd(lookupStage);
	pipeline.unwind("$doc");

	// add parents to the doc.o* field
	// { $set: { "doc.o*": { $setUnion: [ "$doc.o*", "$parents" ] } } }
	pipeline.setUnion("doc.o*", {"$doc.o*", "$directParents"});
	// make the "doc" field the new root
	pipeline.replaceRoot("$doc");
	// merge the result into the collection
	pipeline.merge(collection);
}

void MongoTaxonomy::updateHierarchyP(
		Pipeline &pipeline,
		const std::string_view &collection,
		const std::string_view &relation,
		const std::string_view &newChild) {
	// lookup hierarchy into array field "parents"
	lookupParents(pipeline, collection, newChild, relation);

	// lookup documents that include the child in the p* field
	bson_t lookupArray;
	auto lookupStage = pipeline.appendStageBegin("$lookup");
	BSON_APPEND_UTF8(lookupStage, "from", collection.data());
	BSON_APPEND_UTF8(lookupStage, "as", "doc");
	BSON_APPEND_ARRAY_BEGIN(lookupStage, "pipeline", &lookupArray);
	{
		Pipeline lookupPipeline(&lookupArray);
		// { $match: { "p*": $newChild } }
		auto matchStage = lookupPipeline.appendStageBegin("$match");
		BSON_APPEND_UTF8(matchStage, "p*", newChild.data());
		lookupPipeline.appendStageEnd(matchStage);
		// { $project: { "p*": 1 } }
		lookupPipeline.project("p*");
	}
	bson_append_array_end(lookupStage, &lookupArray);
	pipeline.appendStageEnd(lookupStage);
	pipeline.unwind("$doc");

	// add parents to the doc.p* field
	// { $set: { "doc.p*": { $setUnion: [ "$doc.p*", "$parents" ] } } }
	pipeline.setUnion("doc.p*", {"$doc.p*", "$directParents"});
	// make the "doc" field the new root
	pipeline.replaceRoot("$doc");
	// merge the result into the collection
	pipeline.merge(collection);
}
