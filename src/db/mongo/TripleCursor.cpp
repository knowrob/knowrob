//
// Created by daniel on 07.04.23.
//

#include <boost/algorithm/string.hpp>
#include "knowrob/Logger.h"
#include "knowrob/db/mongo/TripleCursor.h"

using namespace knowrob::mongo;

TripleCursor::TripleCursor(const std::shared_ptr<Collection> &collection)
		: Cursor(collection),
		  tripleDocument_(nullptr),
		  tripleIter_() {
}

bool TripleCursor::nextTriple(FramedTripleView &tripleData) //NOLINT
{
	const bson_oid_t *tripleOID = nullptr;
	return nextTriple(tripleData, &tripleOID);
}

bool TripleCursor::nextTriple(FramedTripleView &tripleData, const bson_oid_t **tripleOID) //NOLINT
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
				tripleData.setAgent(bson_iter_utf8(&tripleIter_, nullptr));
			} else if (key == "uncertain" && bson_iter_bool(&tripleIter_)) {
				tripleData.setEpistemicOperator(EpistemicOperator::BELIEF);
			} else if (key == "occasional" && bson_iter_bool(&tripleIter_)) {
				tripleData.setTemporalOperator(TemporalOperator::SOMETIMES);
			} else if (key == "since") {
				tripleData.setBegin(bson_iter_double(&tripleIter_));
			} else if (key == "until") {
				tripleData.setEnd(bson_iter_double(&tripleIter_));
			} else if (key == "confidence") {
				tripleData.setConfidence(bson_iter_double(&tripleIter_));
				tripleData.setEpistemicOperator(EpistemicOperator::BELIEF);
			} else if (key == "o") {
				switch (bson_iter_type(&tripleIter_)) {
					case BSON_TYPE_UTF8: {
						// TODO: currently mongo KG does not store the type of the literal,
						//        so we cannot trivially distinguish between IRI and literal.
						//        Another option is to require Vocabulary as an argument of TripleCursor.
						auto utf8 = bson_iter_utf8(&tripleIter_, nullptr);
						// if the first letter is an underscore, it's a blank node
						if (utf8[0] == '_') {
							tripleData.setObjectBlank(utf8);
						} else if (boost::algorithm::starts_with(utf8, "http://") ||
								   boost::algorithm::starts_with(utf8, "https://")) {
							// The string is an IRI.
							// Currently we do that by checking if its prefix is "http://" or "https://".
							tripleData.setObjectIRI(utf8);
						} else {
							tripleData.setStringValue(utf8);
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
