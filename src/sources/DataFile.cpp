/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/sources/DataFile.h"

using namespace knowrob;

DataFile::DataFile(const URI &uri, std::string_view format)
		: DataSource(uri, format) {
}
