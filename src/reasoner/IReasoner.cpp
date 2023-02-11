/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/Logger.h"
#include "knowrob/reasoner/IReasoner.h"

using namespace knowrob;

bool IReasoner::hasCapability(ReasonerCapability capability) const
{
    return (getCapabilities() & capability);
}

void IReasoner::addDataSourceHandler(const std::string &format, const DataSourceLoader &fn)
{
	dataSourceHandler_[format] = fn;
}

bool IReasoner::loadDataSource(const DataSourcePtr &dataSource)
{
	if(dataSource->dataFormat().empty()) {
		return loadDataSourceWithUnknownFormat(dataSource);
	}
	else {
		auto it = dataSourceHandler_.find(dataSource->dataFormat());
		return (it != dataSourceHandler_.end()) && it->second(dataSource);
	}
}
