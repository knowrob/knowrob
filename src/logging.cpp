/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

// KnowRob
#include <knowrob/logging.h>
// logging
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>

using namespace knowrob;

struct Logger::impl {
	std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> console_sink;
	std::shared_ptr<spdlog::sinks::rotating_file_sink_mt> file_sink;
	std::shared_ptr<spdlog::logger> logger;
	impl()
	: console_sink(std::make_shared<spdlog::sinks::stdout_color_sink_mt>())
	{}
};

Logger::Logger()
: pimpl_{std::make_unique<impl>()}
{
	updateLogger();
}

void Logger::updateLogger()
{
	if(pimpl_->file_sink)
		pimpl_->logger = std::make_shared<spdlog::logger>(
				"multi_sink", spdlog::sinks_init_list({ pimpl_->console_sink, pimpl_->file_sink }));
	else
		pimpl_->logger = std::make_shared<spdlog::logger>(
				"multi_sink", spdlog::sinks_init_list({ pimpl_->console_sink }));
	spdlog::set_default_logger(pimpl_->logger);
	spdlog::set_level(spdlog::level::trace);
	spdlog::flush_every(std::chrono::seconds(2));
}

Logger& Logger::get()
{
	static Logger singleton;
	return singleton;
}

void Logger::initialize()
{
	setSinkLevel(Console, spdlog::level::info);
	setSinkPattern(Console, "[%$%H:%M:%S.%e] [%^%l%$] %v");
}

void Logger::loadConfiguration(boost::property_tree::ptree &config)
{
	auto &consoleConfig = config.get_child("console-sink");
	if(!consoleConfig.empty()) {
		if(consoleConfig.count("level")) {
			setSinkLevel(Console,
						 spdlog::level::from_str(consoleConfig.get<std::string>("level")));
		}
		if(consoleConfig.count("pattern")) {
			setSinkPattern(Console,
						   consoleConfig.get<std::string>("pattern"));
		}
	}

	auto &fileConfig = config.get_child("file-sink");
	if(!fileConfig.empty()) {
		setupFileSink(fileConfig.get<std::string>("basename", "knowrob.log"),
					  fileConfig.get<bool>("rotate", true),
					  fileConfig.get<uint32_t>("max_size", 1048576),
					  fileConfig.get<uint32_t>("max_files", 4));
		if(fileConfig.count("level")) {
			setSinkLevel(File,
						 spdlog::level::from_str(fileConfig.get<std::string>("level")));
		}
		if(fileConfig.count("pattern")) {
			setSinkPattern(File,
						   fileConfig.get<std::string>("pattern"));
		}
	}

	if(config.count("flush-interval")) {
		spdlog::flush_every(std::chrono::seconds(
				config.get<int64_t>("flush-interval")));
	}
}

void Logger::setupFileSink(const std::string &basename, bool rotate, uint32_t max_size, uint32_t max_files)
{
	auto &self = get();
	self.pimpl_->file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
			basename, max_size, max_files, rotate);
	setSinkLevel(File, spdlog::level::trace);
	setSinkPattern(File, "[%c] [thread:%t] [%^%l%$] %v (%s:%#)");
	self.updateLogger();
}

void Logger::setSinkLevel(SinkType sinkType, spdlog::level::level_enum log_level) {
	switch(sinkType) {
		case Console:
			get().pimpl_->console_sink->set_level(log_level);
			break;
		case File:
			if(get().pimpl_->file_sink)
				get().pimpl_->file_sink->set_level(log_level);
			break;
	}
}

void Logger::setSinkPattern(SinkType sinkType, const std::string &pattern) {
	switch(sinkType) {
		case Console:
			get().pimpl_->console_sink->set_pattern(pattern);
			break;
		case File:
			if(get().pimpl_->file_sink)
				get().pimpl_->file_sink->set_pattern(pattern);
			break;
	}
}