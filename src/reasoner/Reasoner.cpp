/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/Logger.h"
#include "knowrob/reasoner/Reasoner.h"

using namespace knowrob;

Reasoner::Reasoner()
: reasonerManagerID_(0)
{
}

void Reasoner::setReasonerManager(uint32_t managerID)
{
    reasonerManagerID_ = managerID;
}

PredicateDescriptionPtr Reasoner::getLiteralDescription(const RDFLiteral &literal)
{
	if(literal.propertyTerm()->type() == TermType::STRING) {
		auto p = std::static_pointer_cast<StringTerm>(literal.propertyTerm());
		return getDescription(std::make_shared<PredicateIndicator>(p->value(), 2));
	}
	else {
		return {};
	}
}

void Reasoner::addDataHandler(const std::string &format, const std::function<bool(const DataSourcePtr &)> &fn)
{
	dataSourceHandler_[format] = fn;
}

bool Reasoner::loadDataSource(const DataSourcePtr &dataSource)
{
	if(dataSource->dataFormat().empty()) {
		return loadDataSourceWithUnknownFormat(dataSource);
	}
	else {
		auto it = dataSourceHandler_.find(dataSource->dataFormat());
		return (it != dataSourceHandler_.end()) && it->second(dataSource);
	}
}
