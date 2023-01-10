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
		explicit DataFile(const std::string& path, const std::string& format={})
		: DataSource(), path_(path), format_(format) {}
		
		const std::string& path() const { return path_; }
		const std::string& format() const { return format_; }
		bool hasUnknownFormat() const { return format_.empty(); }
	protected:
		std::string path_;
		std::string format_;
	};
	using DataFilePtr = std::shared_ptr<DataFile>;
	
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
