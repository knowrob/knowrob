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

bool TripleCursor::nextTriple(StatementData &tripleData) //NOLINT
{
    if(next(&tripleDocument_) &&
       bson_iter_init(&tripleIter_, tripleDocument_))
    {
        tripleData.documentID = nullptr;
        tripleData.subject = nullptr;
        tripleData.predicate = nullptr;
        tripleData.object = nullptr;
        tripleData.graph = nullptr;
        tripleData.begin = std::nullopt;
        tripleData.begin = std::nullopt;
        tripleData.end = std::nullopt;
        tripleData.objectType = RDF_STRING_LITERAL;

        while(bson_iter_next(&tripleIter_)) {
            std::string_view key = bson_iter_key(&tripleIter_);
            if(key=="_id")
                tripleData.documentID = (void*)bson_iter_oid(&tripleIter_);
            else if(key=="s")
                tripleData.subject = bson_iter_utf8(&tripleIter_, nullptr);
            else if(key=="p")
                tripleData.predicate = bson_iter_utf8(&tripleIter_, nullptr);
            else if(key=="graph")
                tripleData.graph = bson_iter_utf8(&tripleIter_, nullptr);
            else if(key=="a")
                tripleData.agent = bson_iter_utf8(&tripleIter_, nullptr);
            else if(key=="c")
                tripleData.confidence = bson_iter_double(&tripleIter_);
            else if(key=="o") {
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
            }
            else if(key=="scope") {
                bson_iter_t scopeIter, timeIter;
                bson_iter_recurse (&tripleIter_, &scopeIter);
                bson_iter_find(&scopeIter, "time");
                bson_iter_recurse (&scopeIter, &timeIter);

                while(bson_iter_next(&timeIter)) {
                    std::string_view scopeKey = bson_iter_key(&timeIter);
                    if(scopeKey=="since")
                        tripleData.begin = bson_iter_double(&timeIter);
                    else if(scopeKey=="until")
                        tripleData.end = bson_iter_double(&timeIter);
                }
            }
        }

        return true;
    }
    else {
        return false;
    }
}
