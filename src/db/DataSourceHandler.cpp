#include "knowrob/db/DataSourceHandler.h"

using namespace knowrob;

void
DataSourceHandler::addDataHandler(const std::string &format, const std::function<bool(const DataSourcePtr &)> &fn) {
	dataSourceHandler_[format] = fn;
}

bool DataSourceHandler::loadDataSource(const DataSourcePtr &dataSource) {
	if (dataSource->format().empty()) {
		return loadDataSourceWithUnknownFormat(dataSource);
	} else {
		auto it = dataSourceHandler_.find(dataSource->format());
		if (it != dataSourceHandler_.end()) {
			return it->second(dataSource);
		} else {
			return false;
		}
	}
}

bool DataSourceHandler::hasDataHandler(const DataSourcePtr &dataSource) const {
	return dataSourceHandler_.find(dataSource->format()) != dataSourceHandler_.end();
}
