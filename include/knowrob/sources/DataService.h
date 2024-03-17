/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_DATA_SERVICE_H
#define KNOWROB_DATA_SERVICE_H

#include "knowrob/sources/DataSource.h"

namespace knowrob {
	/**
	 * A data service is a data source that provides data in through a querying service.
	 */
	class DataService : public DataSource {
	public:
		/**
		 * @param uri URI of the data source.
		 * @param format string identifier of the data format.
		 */
		DataService(const URI &uri, std::string_view format);
	};

} // knowrob

#endif //KNOWROB_DATA_SERVICE_H
