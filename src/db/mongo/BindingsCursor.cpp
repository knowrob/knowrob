/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/Logger.h"
#include "knowrob/db/mongo/BindingsCursor.h"
#include "knowrob/terms/Numeric.h"
#include "knowrob/terms/IRIAtom.h"
#include "knowrob/terms/Blank.h"
#include "knowrob/db/mongo/bson-helper.h"

using namespace knowrob;
using namespace knowrob::mongo;

BindingsCursor::BindingsCursor(const std::shared_ptr<Collection> &collection)
		: Cursor(collection),
		  resultDocument_(nullptr),
		  resultIter_(),
		  timeIter_(),
		  varIter_(),
		  valIter_(),
		  scopeIter_() {
}

void BindingsCursor::setSubstitution(const std::shared_ptr<Bindings> &bindings) {
	while (bson_iter_next(&varIter_)) {
		auto var = std::make_shared<Variable>(bson_iter_key(&varIter_));

		if (!bson_iter_recurse(&varIter_, &valIter_)) continue;
		if (!bson_iter_find(&valIter_, "val")) continue;

		// read the value of the variable
		switch (bson_iter_type(&valIter_)) {
			case BSON_TYPE_UTF8: {
				auto utf8 = bson_iter_utf8(&valIter_, nullptr);
				// note: currently mongo KG does not store the type of the literal,
				// so we cannot trivially distinguish between IRI and literal and need to guess here.
				switch (rdfNodeTypeGuess(utf8)) {
					case RDFNodeType::BLANK:
						bindings->set(var, std::make_shared<Blank>(utf8));
						break;
					case RDFNodeType::IRI:
						bindings->set(var, std::make_shared<IRIAtom>(utf8));
						break;
					case RDFNodeType::LITERAL:
						bindings->set(var, std::make_shared<StringView>(utf8));
						break;
				}
				break;
			}
			case BSON_TYPE_INT32:
				bindings->set(var, std::make_shared<Integer>(bson_iter_int32(&valIter_)));
				break;
			case BSON_TYPE_INT64:
				bindings->set(var, std::make_shared<Long>(bson_iter_int64(&valIter_)));
				break;
			case BSON_TYPE_BOOL:
				bindings->set(var, std::make_shared<Boolean>(bson_iter_bool(&valIter_)));
				break;
			case BSON_TYPE_DOUBLE:
				bindings->set(var, std::make_shared<Double>(bson_iter_double(&valIter_)));
				break;
			default:
				KB_WARN("unsupported type {} for predicate arguments.", bson_iter_type(&valIter_));
				break;
		}
	}
}

bool BindingsCursor::nextBindings(const std::shared_ptr<Bindings> &bindings) {
	if (!next(&resultDocument_)) return false;
	if (!bson_iter_init(&resultIter_, resultDocument_)) return false;

	while (bson_iter_next(&resultIter_)) {
		std::string_view resultKey(bson_iter_key(&resultIter_));
		// read variables from "v_VARS". each sub-field is named according to the field of
		// a variable.
		if (resultKey == "v_VARS" && bson_iter_recurse(&resultIter_, &varIter_)) {
			setSubstitution(bindings);
		}
	}

	return true;
}
