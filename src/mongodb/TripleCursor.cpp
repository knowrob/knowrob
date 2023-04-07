//
// Created by daniel on 07.04.23.
//

#include "knowrob/Logger.h"
#include "knowrob/mongodb/TripleCursor.h"

using namespace knowrob::mongo;

TripleCursor::TripleCursor(const std::shared_ptr<Collection> &collection)
: Cursor(collection),
  tripleDocument_(nullptr),
  tripleIter_()
{
}

bool TripleCursor::nextTriple(TripleData &tripleData) //NOLINT
{
    if(next(&tripleDocument_) &&
       bson_iter_init(&tripleIter_, tripleDocument_))
    {
        if(!bson_iter_find(&tripleIter_, "s")) {
            KB_WARN("skipping corrupt triple: missing \"s\" field.");
            return nextTriple(tripleData);
        }
        tripleData.subject = bson_iter_utf8(&tripleIter_, nullptr);

        if(!bson_iter_find(&tripleIter_, "p")) {
            KB_WARN("skipping corrupt triple: missing \"p\" field.");
            return nextTriple(tripleData);
        }
        tripleData.predicate = bson_iter_utf8(&tripleIter_, nullptr);

        if(!bson_iter_find(&tripleIter_, "o")) {
            KB_WARN("skipping corrupt triple: missing \"o\" field.");
            return nextTriple(tripleData);
        }

        switch(bson_iter_type(&tripleIter_)) {
            case BSON_TYPE_UTF8:
                tripleData.object = bson_iter_utf8(&tripleIter_, nullptr);
                tripleData.objectType = RDF_STRING_LITERAL;
                break;
            case BSON_TYPE_DOUBLE:
                tripleData.objectDouble = bson_iter_double(&tripleIter_);
                tripleData.objectType = RDF_DOUBLE_LITERAL;
                break;
            case BSON_TYPE_BOOL:
                tripleData.objectInteger = bson_iter_bool(&tripleIter_);
                tripleData.objectType = RDF_BOOLEAN_LITERAL;
                break;
            case BSON_TYPE_INT32:
                tripleData.objectInteger = bson_iter_int32(&tripleIter_);
                tripleData.objectType = RDF_INT64_LITERAL;
                break;
            case BSON_TYPE_INT64:
                tripleData.objectInteger = bson_iter_int64(&tripleIter_);
                tripleData.objectType = RDF_INT64_LITERAL;
                break;
            default:
                KB_WARN("skipping triple with unexpected type '{}' of \"o\" field.", bson_iter_type(&tripleIter_));
                return nextTriple(tripleData);
        }
        return true;
    }
    else {
        return false;
    }
}
