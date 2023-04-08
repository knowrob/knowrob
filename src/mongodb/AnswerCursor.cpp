//
// Created by daniel on 08.04.23.
//

#include "knowrob/Logger.h"
#include "knowrob/mongodb/AnswerCursor.h"

using namespace knowrob::mongo;

AnswerCursor::AnswerCursor(const std::shared_ptr<Collection> &collection)
: Cursor(collection),
  resultDocument_(nullptr),
  resultIter_(),
  varIter_(),
  valIter_()
{
}

bool AnswerCursor::nextAnswer(const std::shared_ptr<Answer> &answer)
{
    if(!next(&resultDocument_)) return false;
    if(!bson_iter_init(&resultIter_, resultDocument_)) return false;

    // read variables from "v_VARS". each sub-field is named according to the field of
    // a variable.
    if(!bson_iter_find(&resultIter_, "v_VARS") ||
       !bson_iter_recurse(&resultIter_, &varIter_))
    {
        KB_WARN("missing \"v_VARS\" field in result document.");
        return false;
    }

    while(bson_iter_next (&varIter_)) {
        Variable var(bson_iter_key(&varIter_));

        if(!bson_iter_recurse(&varIter_, &valIter_)) continue;
        if(!bson_iter_find(&valIter_, "val")) continue;

        // read the value of the variable
        switch(bson_iter_type(&valIter_)) {
            case BSON_TYPE_UTF8:
                answer->substitute(var, std::make_shared<StringTerm>(bson_iter_utf8(&valIter_,nullptr)));
                break;
            case BSON_TYPE_INT32:
                answer->substitute(var, std::make_shared<Integer32Term>(bson_iter_int32(&valIter_)));
                break;
            case BSON_TYPE_INT64:
                answer->substitute(var, std::make_shared<LongTerm>(bson_iter_int64(&valIter_)));
                break;
            case BSON_TYPE_BOOL:
                answer->substitute(var, std::make_shared<Integer32Term>(bson_iter_bool(&valIter_)));
                break;
            case BSON_TYPE_DOUBLE:
                answer->substitute(var, std::make_shared<DoubleTerm>(bson_iter_double(&valIter_)));
                break;
            default:
                KB_WARN("unsupported type {} for predicate arguments.", bson_iter_type(&valIter_));
                break;
        }
    }

    return true;
}
