/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/sources/DataService.h"

using namespace knowrob;

DataService::DataService(const URI &uri, std::string_view format)
		: DataSource(uri, format) {
}
