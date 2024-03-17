/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_DATA_FILE_H
#define KNOWROB_DATA_FILE_H

#include "knowrob/sources/DataSource.h"

namespace knowrob {
	/**
	 * Represents a data source that is a file.
	 */
	class DataFile : public DataSource {
	public:
		/**
		 * @param uri URI of the data source.
		 * @param format string identifier of the data format.
		 */
		DataFile(const URI &uri, std::string_view format);
	};

} // knowrob

#endif //KNOWROB_DATA_FILE_H
