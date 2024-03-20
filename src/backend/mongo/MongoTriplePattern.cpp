/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/backend/mongo/MongoTriplePattern.h"
#include "knowrob/backend/mongo/MongoTerm.h"

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
