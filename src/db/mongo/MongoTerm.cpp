/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/db/mongo/MongoTerm.h"
#include "knowrob/terms/ListTerm.h"
#include "knowrob/terms/Numeric.h"
#include "knowrob/terms/Variable.h"

using namespace knowrob;

void MongoTerm::append( //NOLINT
		bson_t *doc,
		const char *key,
		const TermPtr &term,
		const char *queryOperator,
		bool matchNullValues,
		bool includeVariables) {
	bson_t queryOperatorDoc;
	bson_t orArray, orCase1, orCase2;
	bson_t *valueDocument;
	const char *valueKey;

	if (matchNullValues) {
		// allow to pass through if key is undefined. e.g. important for time scope etc.
		// e.g. {$or: [ { b: null }, { b: {$gt: 10.0} } ]}
		BSON_APPEND_ARRAY_BEGIN(doc, "$or", &orArray);

		BSON_APPEND_DOCUMENT_BEGIN(&orArray, "0", &orCase1);
		BSON_APPEND_NULL(&orCase1, key);
		bson_append_document_end(&orArray, &orCase1);

		BSON_APPEND_DOCUMENT_BEGIN(&orArray, "1", &orCase2);
	}

	if (queryOperator) {
		BSON_APPEND_DOCUMENT_BEGIN(matchNullValues ? &orCase2 : doc, key, &queryOperatorDoc);
		valueDocument = &queryOperatorDoc;
		valueKey = queryOperator;
	} else {
		valueDocument = (matchNullValues ? &orCase2 : doc);
		valueKey = key;
	}

	if (term->termType() == TermType::ATOMIC) {
		auto atomic = std::static_pointer_cast<Atomic>(term);
		switch (atomic->atomicType()) {
			case AtomicType::STRING:
			case AtomicType::ATOM:
				BSON_APPEND_UTF8(valueDocument, valueKey, atomic->stringForm().data());
				break;
			case AtomicType::NUMERIC: {
				auto numeric = std::static_pointer_cast<Numeric>(atomic);
				switch (numeric->xsdType()) {
					case XSDType::FLOAT:
					case XSDType::DOUBLE:
						BSON_APPEND_DOUBLE(valueDocument, valueKey, numeric->asDouble());
						break;
					case XSDType::NON_NEGATIVE_INTEGER:
					case XSDType::UNSIGNED_INT:
					case XSDType::INTEGER:
						BSON_APPEND_INT32(valueDocument, valueKey, numeric->asInteger());
						break;
					case XSDType::UNSIGNED_LONG:
					case XSDType::LONG:
						BSON_APPEND_INT64(valueDocument, valueKey, numeric->asLong());
						break;
					case XSDType::UNSIGNED_SHORT:
					case XSDType::SHORT:
						BSON_APPEND_INT32(valueDocument, valueKey, numeric->asShort());
						break;
					case XSDType::BOOLEAN:
						BSON_APPEND_BOOL(valueDocument, valueKey, numeric->asBoolean());
						break;
					case XSDType::STRING:
					case XSDType::LAST:
						break;
				}
				break;
			}
		}
	} else if (term->termType() == TermType::FUNCTION) {
		append(valueDocument, valueKey,
			   ((ListTerm *) term.get())->elements());
	} else if (includeVariables && term->termType() == TermType::VARIABLE) {
		static const std::string varPrefix("$");
		auto varKey = MongoTerm::variableKey(std::static_pointer_cast<Variable>(term)->name());
		BSON_APPEND_UTF8(valueDocument, valueKey, (varPrefix + varKey).data());
	}

	if (queryOperator) {
		bson_append_document_end(matchNullValues ? &orCase2 : doc, &queryOperatorDoc);
	}
	if (matchNullValues) {
		bson_append_document_end(&orArray, &orCase2);
		bson_append_array_end(doc, &orArray);
	}
}

void MongoTerm::appendWithVars(bson_t *doc, const char *key, const TermPtr &term, const char *queryOperator,
							   bool matchNullValue) {
	append(doc, key, term, queryOperator, matchNullValue, true);
}

void MongoTerm::append( // NOLINT
		bson_t *doc,
		const char *key,
		const std::vector<TermPtr> &terms,
		const char *arrayOperator) {
	bson_t orOperator, orArray;
	char arrIndexStr[16];
	const char *arrIndexKey;
	uint32_t arrIndex = 0;

	BSON_APPEND_DOCUMENT_BEGIN(doc, key, &orOperator);
	BSON_APPEND_ARRAY_BEGIN(&orOperator, arrayOperator, &orArray);
	for (auto &term: terms) {
		bson_uint32_to_string(arrIndex++,
							  &arrIndexKey, arrIndexStr, sizeof arrIndexStr);
		append(&orArray, arrIndexKey, term);
	}
	bson_append_array_end(&orOperator, &orArray);
	bson_append_document_end(doc, &orOperator);
}

std::string MongoTerm::variableKey(const std::string_view &varName) {
	std::stringstream ss;
	ss << "v_VARS." << varName << ".val";
	return ss.str();
}
