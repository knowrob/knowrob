/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/db/DataFile.h"

using namespace knowrob;

DataFile::DataFile(const URI &uri, std::string_view format)
		: DataSource(uri, format) {
}
