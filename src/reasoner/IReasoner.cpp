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

void IReasoner::addDataFileHandler(const std::string &format, const DataFileLoader &fn)
{
	dataFileHandler_[format] = fn;
}

bool IReasoner::loadDataFile(const DataFilePtr &dataFile)
{
	if(dataFile->hasUnknownFormat()) {
		return loadDataFileWithUnknownFormat(dataFile);
	}
	else {
		auto it = dataFileHandler_.find(dataFile->format());
		if(it == dataFileHandler_.end()) {
			KB_WARN("Ignoring data file with unknown format \"{}\"", dataFile->format());
			return false;
		}
		else {
			KB_INFO("Using data file {} with format \"{}\".", dataFile->path(), dataFile->format());
			return it->second(dataFile);
		}
	}
}
