
#include <utility>

#include "knowrob/DataSourceHandler.h"

using namespace knowrob;

void DataSourceHandler::addDataHandler(const std::string &format, const std::function<bool(const DataSourcePtr &)> &fn)
{
	dataSourceHandler_[format] = fn;
}

bool DataSourceHandler::loadDataSource(const DataSourcePtr &dataSource)
{
	if(dataSource->dataFormat().empty()) {
		return loadDataSourceWithUnknownFormat(dataSource);
	}
	else {
		auto it = dataSourceHandler_.find(dataSource->dataFormat());
		return (it != dataSourceHandler_.end()) && it->second(dataSource);
	}
}
