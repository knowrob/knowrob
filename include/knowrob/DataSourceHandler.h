/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_DATA_SOURCE_HANDLER_H_
#define KNOWROB_DATA_SOURCE_HANDLER_H_

#include "map"
#include "string"
#include "functional"
#include "knowrob/DataSource.h"

namespace knowrob {
	/**
	 * An object that can load data sources.
	 */
	class DataSourceHandler {
    public:
		/**
		 * Add a handler for a data source format.
		 * @param format the format name.
		 * @param fn the handler function.
		 */
		void addDataHandler(const std::string &format,
							const std::function<bool(const DataSourcePtr &)> &fn);

		/**
		 * Load a data source.
		 * The knowledge base system calls this function for each data source
		 * that is passed to the reasoner.
		 * @param dataSource the data source to load.
		 */
		bool loadDataSource(const DataSourcePtr &dataSource);


    protected:
		std::map<std::string, DataSourceLoader> dataSourceHandler_;

		virtual bool loadDataSourceWithUnknownFormat(const DataSourcePtr&) { return false; }
	};
}

#endif //KNOWROB_DATA_SOURCE_HANDLER_H_
