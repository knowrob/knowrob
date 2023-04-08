//
// Created by daniel on 08.04.23.
//


#include "knowrob/mongodb/aggregation/terms.h"
#include "knowrob/terms/Constant.h"
#include "knowrob/terms/ListTerm.h"

using namespace knowrob;
using namespace knowrob::mongo;

void aggregation::appendTermQuery( //NOLINT
        bson_t *doc,
        const char *key,
        const TermPtr &term,
        const char *queryOperator)
{
    bson_t queryOperatorDoc;
    bson_t *valueDocument;
    const char *valueKey;

    if(queryOperator) {
        BSON_APPEND_DOCUMENT_BEGIN(doc, key, &queryOperatorDoc);
        valueDocument = &queryOperatorDoc;
        valueKey = queryOperator;
    }
    else {
        valueDocument = doc;
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
        case TermType::MODAL_OPERATOR:
        case TermType::PREDICATE:
            break;
    }

    if(queryOperator) {
        bson_append_document_end(doc, &queryOperatorDoc);
    }
}

void aggregation::appendArrayQuery( // NOLINT
        bson_t *doc, const char *key, const std::vector<TermPtr> &terms)
{
    bson_t orOperator, orArray;
    BSON_APPEND_DOCUMENT_BEGIN(doc, key, &orOperator);
    BSON_APPEND_ARRAY_BEGIN(&orOperator, "$or", &orArray);
    uint32_t arrayIndex = 0;
    for(auto &term : terms) {
        auto indexKey = std::to_string(arrayIndex++);
        appendTermQuery(doc, indexKey.c_str(), term);
        //BSON_APPEND_UTF8(&orArray, indexKey.c_str(), entity);
    }
    bson_append_array_end(&orOperator, &orArray);
    bson_append_document_end(doc, &orOperator);
}
