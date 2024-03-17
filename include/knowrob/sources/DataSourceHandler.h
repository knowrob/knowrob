/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_DATA_SOURCE_HANDLER_H_
#define KNOWROB_DATA_SOURCE_HANDLER_H_

#include "map"
#include "string"
#include "functional"
#include "DataSource.h"

namespace knowrob {
	using DataSourceLoader = std::function<bool(const DataSourcePtr &)>;

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
		void addDataHandler(const std::string &format, const DataSourceLoader &fn);

		/**
		 * Load a data source.
		 * The knowledge base system calls this function for each data source
		 * that is passed to the reasoner.
		 * @param dataSource the data source to load.
		 */
		bool loadDataSource(const DataSourcePtr &dataSource);

		/**
		 * Check if a handler for a data source format is available.
		 * @param format the format name.
		 * @return true if a handler is available.
		 */
		bool hasDataHandler(const DataSourcePtr &dataSource) const;


	protected:
		std::map<std::string, DataSourceLoader> dataSourceHandler_;

		virtual bool loadDataSourceWithUnknownFormat(const DataSourcePtr &) { return false; }
	};
}

#endif //KNOWROB_DATA_SOURCE_HANDLER_H_
