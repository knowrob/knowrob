/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_DATA_SOURCES_H__
#define __KNOWROB_DATA_SOURCES_H__

namespace knowrob {
	class DataSource {
	};
	
	class DataFile : public DataSource {
	public:
		DataFile(const std::string& path)
		: DataSource(), path_(path) {}
		
		const std::string& path() const { return path_; }
	protected:
		std::string path_;
	};
	
	class FactBase : public DataSource {
	public:
		FactBase() : DataSource() {};
	};
	
	class RuleBase : public DataSource {
	public:
		RuleBase() : DataSource() {};
	};
}

#endif //__KNOWROB_DATA_SOURCES_H__
