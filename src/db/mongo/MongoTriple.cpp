/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/db/mongo/MongoTriple.h"

using namespace knowrob::mongo;
using namespace knowrob;

MongoTriple::MongoTriple(const semweb::VocabularyPtr &vocabulary,
						 const FramedTriple &tripleData,
						 const std::string &fallbackOrigin,
						 bool isTaxonomic)
		: document_(createDocument(vocabulary, tripleData, fallbackOrigin, isTaxonomic)) {
}

static inline void appendXSDLiteral(bson_t *tripleDoc, const FramedTriple &tripleData) {
	auto xsdType = tripleData.xsdType().has_value() ? tripleData.xsdType().value() : XSDType::STRING;
	switch (xsdType) {
		case XSDType::STRING:
			BSON_APPEND_UTF8(tripleDoc, "o", tripleData.valueAsString().data());
			break;
		case XSDType::DOUBLE:
			BSON_APPEND_DOUBLE(tripleDoc, "o", tripleData.valueAsDouble());
			break;
		case XSDType::FLOAT:
			BSON_APPEND_DOUBLE(tripleDoc, "o", tripleData.valueAsFloat());
			break;
		case XSDType::LONG:
			BSON_APPEND_INT64(tripleDoc, "o", tripleData.valueAsLong());
			break;
		case XSDType::NON_NEGATIVE_INTEGER:
		case XSDType::INTEGER:
			BSON_APPEND_INT32(tripleDoc, "o", tripleData.valueAsInt());
			break;
		case XSDType::SHORT:
			BSON_APPEND_INT32(tripleDoc, "o", tripleData.valueAsShort());
			break;
		case XSDType::BOOLEAN:
			BSON_APPEND_BOOL(tripleDoc, "o", tripleData.valueAsBoolean());
			break;
		case XSDType::UNSIGNED_INT:
			BSON_APPEND_INT32(tripleDoc, "o", tripleData.valueAsUnsignedInt());
			break;
		case XSDType::UNSIGNED_SHORT:
			BSON_APPEND_INT32(tripleDoc, "o", tripleData.valueAsUnsignedShort());
			break;
		case XSDType::UNSIGNED_LONG:
			BSON_APPEND_INT64(tripleDoc, "o", tripleData.valueAsUnsignedLong());
			break;
		case XSDType::LAST:
			break;
	}
}

bson_t *MongoTriple::createDocument(
		const semweb::VocabularyPtr &vocabulary,
		const FramedTriple &tripleData,
		const std::string &fallbackOrigin,
		bool isTaxonomic) {
	bson_t parentsArray;
	uint32_t arrIndex = 0;
	auto counterPtr = &arrIndex;

	bson_t *tripleDoc = bson_new();
	BSON_APPEND_UTF8(tripleDoc, "s", tripleData.subject().data());
	BSON_APPEND_UTF8(tripleDoc, "p", tripleData.predicate().data());

	if (isTaxonomic) {
		if (tripleData.isObjectIRI() || tripleData.isObjectBlank()) {
			auto objectIRI = tripleData.valueAsString();
			BSON_APPEND_UTF8(tripleDoc, "o", objectIRI.data());
			// also create a field "o*" with the parents of the object
			BSON_APPEND_ARRAY_BEGIN(tripleDoc, "o*", &parentsArray);
			auto parentsPtr = &parentsArray;
			if (vocabulary->isDefinedProperty(objectIRI)) {
				vocabulary->getDefinedProperty(objectIRI)->forallParents(
						[parentsPtr, counterPtr](const auto &parent) {
							auto counterKey = std::to_string((*counterPtr)++);
							BSON_APPEND_UTF8(parentsPtr, counterKey.c_str(), parent.iri().data());
						});
			} else if (vocabulary->isDefinedClass(objectIRI)) {
				// read parents array
				vocabulary->getDefinedClass(objectIRI)->forallParents(
						[parentsPtr, counterPtr](const auto &parent) {
							auto counterKey = std::to_string((*counterPtr)++);
							BSON_APPEND_UTF8(parentsPtr, counterKey.c_str(), parent.iri().data());
						});
			} else {
				BSON_APPEND_UTF8(&parentsArray, "0", objectIRI.data());
			}
			bson_append_array_end(tripleDoc, &parentsArray);
		} else {
			appendXSDLiteral(tripleDoc, tripleData);
		}
	} else {
		if (tripleData.isObjectIRI() || tripleData.isObjectBlank()) {
			BSON_APPEND_UTF8(tripleDoc, "o", tripleData.valueAsString().data());
		} else {
			appendXSDLiteral(tripleDoc, tripleData);
		}
		// read parents array
		BSON_APPEND_ARRAY_BEGIN(tripleDoc, "p*", &parentsArray);
		auto parentsPtr = &parentsArray;
		vocabulary->defineProperty(tripleData.predicate())->forallParents(
				[parentsPtr, counterPtr](const auto &parent) {
					auto counterKey = std::to_string((*counterPtr)++);
					BSON_APPEND_UTF8(parentsPtr, counterKey.c_str(), parent.iri().data());
				});
		bson_append_array_end(tripleDoc, &parentsArray);
	}

	if (tripleData.graph()) {
		BSON_APPEND_UTF8(tripleDoc, "graph", tripleData.graph().value().data());
	} else {
		BSON_APPEND_UTF8(tripleDoc, "graph", fallbackOrigin.c_str());
	}

	if (tripleData.agent())
		BSON_APPEND_UTF8(tripleDoc, "agent", tripleData.agent().value().data());

	bool isBelief;
	if (tripleData.confidence().has_value()) {
		BSON_APPEND_DOUBLE(tripleDoc, "confidence", tripleData.confidence().value());
		isBelief = true;
	} else {
		isBelief = tripleData.isUncertain();
	}
	if (isBelief) {
		// flag the statement as "uncertain"
		BSON_APPEND_BOOL(tripleDoc, "uncertain", true);
	}

	if (tripleData.isOccasional()) {
		// flag the statement as "occasional", meaning it is only known that it was true at some past instants
		BSON_APPEND_BOOL(tripleDoc, "occasional", true);
	}

	if (tripleData.begin().has_value() || tripleData.end().has_value()) {
		bson_t scopeDoc, timeDoc;
		BSON_APPEND_DOCUMENT_BEGIN(tripleDoc, "scope", &scopeDoc);
		BSON_APPEND_DOCUMENT_BEGIN(&scopeDoc, "time", &timeDoc);
		if (tripleData.begin().has_value()) BSON_APPEND_DOUBLE(&timeDoc, "since", tripleData.begin().value());
		if (tripleData.end().has_value()) BSON_APPEND_DOUBLE(&timeDoc, "until", tripleData.end().value());
		bson_append_document_end(&scopeDoc, &timeDoc);
		bson_append_document_end(tripleDoc, &scopeDoc);
	}

	return tripleDoc;
}
