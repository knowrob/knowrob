/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <clocale>
#include "knowrob/backend/mongo/bson-helper.h"

namespace knowrob::mongo {
	std::optional<double> bson_iterOptionalDouble(const bson_iter_t *iter) {
		auto iterType = bson_iter_type(iter);
		if (iterType == BSON_TYPE_DECIMAL128) {
			bson_decimal128_t decimal;
			bson_iter_decimal128(iter, &decimal);
			if (decimal.high == 0x7800000000000000 || decimal.high == 0xf800000000000000) {
				// infinity
				return std::nullopt;
			} else {
				char buffer[BSON_DECIMAL128_STRING];
				bson_decimal128_to_string(&decimal, buffer);
				setlocale(LC_NUMERIC, "C");
				auto val = atof(buffer);
				setlocale(LC_NUMERIC, "");
				return val;
			}
		} else if (iterType == BSON_TYPE_DOUBLE) {
			return bson_iter_double(iter);
		} else {
			return std::nullopt;
		}
	}
}
