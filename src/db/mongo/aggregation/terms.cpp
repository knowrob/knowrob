//
// Created by daniel on 08.04.23.
//


#include "knowrob/db/mongo/aggregation/terms.h"
#include "knowrob/terms/Constant.h"
#include "knowrob/terms/ListTerm.h"
#include "knowrob/Logger.h"

using namespace knowrob;
using namespace knowrob::mongo;

void aggregation::appendTermQuery( //NOLINT
        bson_t *doc,
        const char *key,
        const TermPtr &term,
        const char *queryOperator,
        bool matchNullValues)
{
    bson_t queryOperatorDoc;
    bson_t orArray, orCase1, orCase2;
    bson_t *valueDocument;
    const char *valueKey;

    if(matchNullValues) {
        // allow to pass through if key is undefined. e.g. important for time scope etc.
        // e.g. {$or: [ { b: null }, { b: {$gt: 10.0} } ]}
        BSON_APPEND_ARRAY_BEGIN(doc, "$or", &orArray);

        BSON_APPEND_DOCUMENT_BEGIN(&orArray, "0", &orCase1);
        BSON_APPEND_NULL(&orCase1, key);
        bson_append_document_end(&orArray, &orCase1);

        BSON_APPEND_DOCUMENT_BEGIN(&orArray, "1", &orCase2);
    }

    if(queryOperator) {
        BSON_APPEND_DOCUMENT_BEGIN(matchNullValues ? &orCase2 : doc, key, &queryOperatorDoc);
        valueDocument = &queryOperatorDoc;
        valueKey = queryOperator;
    }
    else {
        valueDocument = (matchNullValues ? &orCase2 : doc);
        valueKey = key;
    }

    switch(term->type()) {
        case TermType::STRING:
            BSON_APPEND_UTF8(valueDocument, valueKey,
                             ((StringTerm*)term.get())->value().c_str());
            break;
        case TermType::VARIABLE:
            break;
        case TermType::DOUBLE:
            BSON_APPEND_DOUBLE(valueDocument, valueKey,
                               ((DoubleTerm*)term.get())->value());
            break;
        case TermType::INT32:
            BSON_APPEND_INT32(valueDocument, valueKey,
                              ((Integer32Term*)term.get())->value());
            break;
        case TermType::LONG:
            BSON_APPEND_INT64(valueDocument, valueKey,
                              ((LongTerm*)term.get())->value());
            break;
        case TermType::LIST:
            appendArrayQuery(valueDocument, valueKey,
                             ((ListTerm*)term.get())->elements());
            break;
        case TermType::PREDICATE:
            break;
    }

    if(queryOperator) {
        bson_append_document_end(matchNullValues ? &orCase2 : doc, &queryOperatorDoc);
    }
    if(matchNullValues) {
        bson_append_document_end(&orArray, &orCase2);
        bson_append_array_end(doc, &orArray);
    }
}

void aggregation::appendArrayQuery( // NOLINT
        bson_t *doc,
        const char *key,
        const std::vector<TermPtr> &terms,
        const char *arrayOperator)
{
    bson_t orOperator, orArray;
    char arrIndexStr[16];
    const char *arrIndexKey;
    uint32_t arrIndex = 0;

    BSON_APPEND_DOCUMENT_BEGIN(doc, key, &orOperator);
    BSON_APPEND_ARRAY_BEGIN(&orOperator, arrayOperator, &orArray);
    for(auto &term : terms) {
        bson_uint32_to_string(arrIndex++,
            &arrIndexKey, arrIndexStr, sizeof arrIndexStr);
		KB_INFO("array elem {} = {}", arrIndexKey, *term);
        appendTermQuery(&orArray, arrIndexKey, term);
    }
    bson_append_array_end(&orOperator, &orArray);
    bson_append_document_end(doc, &orOperator);
}
