/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <boost/algorithm/string.hpp>
#include "knowrob/Logger.h"
#include "knowrob/backend/mongo/TripleCursor.h"
#include "knowrob/terms/RDFNode.h"
#include "knowrob/backend/mongo/bson-helper.h"

using namespace knowrob::mongo;

TripleCursor::TripleCursor(const std::shared_ptr<Collection> &collection)
		: Cursor(collection),
		  tripleDocument_(nullptr),
		  tripleIter_() {
}

bool TripleCursor::nextTriple(FramedTriple &tripleData) //NOLINT
{
	const bson_oid_t *tripleOID = nullptr;
	return nextTriple(tripleData, &tripleOID);
}

bool TripleCursor::nextTriple(FramedTriple &tripleData, const bson_oid_t **tripleOID) //NOLINT
{
	if (next(&tripleDocument_) &&
		bson_iter_init(&tripleIter_, tripleDocument_)) {
		tripleData.reset();

		while (bson_iter_next(&tripleIter_)) {
			std::string_view key = bson_iter_key(&tripleIter_);
			if (key == "_id") {
				*tripleOID = bson_iter_oid(&tripleIter_);
			} else if (key == "s") {
				tripleData.setSubject(bson_iter_utf8(&tripleIter_, nullptr));
			} else if (key == "p") {
				tripleData.setPredicate(bson_iter_utf8(&tripleIter_, nullptr));
			} else if (key == "graph") {
				tripleData.setGraph(bson_iter_utf8(&tripleIter_, nullptr));
			} else if (key == "agent") {
				tripleData.setPerspective(bson_iter_utf8(&tripleIter_, nullptr));
			} else if (key == "uncertain") {
				tripleData.setIsUncertain(bson_iter_bool(&tripleIter_));
			} else if (key == "occasional") {
				tripleData.setIsOccasional(bson_iter_bool(&tripleIter_));
			} else if (key == "since") {
				tripleData.setBegin(bson_iter_double(&tripleIter_));
			} else if (key == "until") {
				tripleData.setEnd(bson_iter_double(&tripleIter_));
			} else if (key == "confidence") {
				tripleData.setConfidence(bson_iter_double(&tripleIter_));
				tripleData.setIsUncertain(true);
			} else if (key == "o") {
				switch (bson_iter_type(&tripleIter_)) {
					case BSON_TYPE_UTF8: {
						// note: currently mongo KG does not store the type of the literal,
						// so we cannot trivially distinguish between IRI and literal and need to guess here.
						auto utf8 = bson_iter_utf8(&tripleIter_, nullptr);
						switch (rdfNodeTypeGuess(utf8)) {
							case RDFNodeType::IRI:
								tripleData.setObjectIRI(utf8);
								break;
							case RDFNodeType::LITERAL:
								tripleData.setStringValue(utf8);
								break;
							case RDFNodeType::BLANK:
								tripleData.setObjectBlank(utf8);
								break;
						}
						break;
					}
					case BSON_TYPE_DOUBLE:
						tripleData.setDoubleValue(bson_iter_double(&tripleIter_));
						break;
					case BSON_TYPE_BOOL:
						tripleData.setIntValue(bson_iter_bool(&tripleIter_));
						break;
					case BSON_TYPE_INT32:
						tripleData.setIntValue(bson_iter_int32(&tripleIter_));
						break;
					case BSON_TYPE_INT64:
						tripleData.setLongValue(bson_iter_int64(&tripleIter_));
						break;
					default:
						KB_WARN("skipping triple with unexpected type '{}' of \"o\" field.",
								bson_iter_type(&tripleIter_));
						return nextTriple(tripleData);
				}
			} else if (key == "scope") {
				bson_iter_t scopeIter, timeIter;
				bson_iter_recurse(&tripleIter_, &scopeIter);
				bson_iter_find(&scopeIter, "time");
				bson_iter_recurse(&scopeIter, &timeIter);

				while (bson_iter_next(&timeIter)) {
					std::string_view scopeKey = bson_iter_key(&timeIter);
					if (scopeKey == "since") {
						auto v = bson_iterOptionalDouble(&timeIter);
						if (v != 0) tripleData.setBegin(v.value());
					} else if (scopeKey == "until") {
						auto v = bson_iterOptionalDouble(&timeIter);
						if (v != 0) tripleData.setEnd(v.value());
					}
				}
			}
		}

		return true;
	} else {
		return false;
	}
}
