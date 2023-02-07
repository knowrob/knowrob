/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

// KnowRob
#include <knowrob/Logger.h>
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
	auto consoleConfig = config.get_child_optional("console-sink");
	if(consoleConfig) {
		auto level = consoleConfig.value().get_optional<std::string>("level");
		auto pattern = consoleConfig.value().get_optional<std::string>("pattern");
		if(level.has_value()) {
			setSinkLevel(Console, spdlog::level::from_str(level.value()));
		}
		if(pattern.has_value()) {
			setSinkPattern(Console, pattern.value());
		}
	}

	auto fileConfig = config.get_child_optional("file-sink");
	if(fileConfig) {
		auto &fileConfig0 = fileConfig.value();
		setupFileSink(fileConfig0.get<std::string>("basename", "knowrob.log"),
					  fileConfig0.get<bool>("rotate", true),
					  fileConfig0.get<uint32_t>("max_size", 1048576),
					  fileConfig0.get<uint32_t>("max_files", 4));

		auto level = fileConfig0.get_optional<std::string>("level");
		auto pattern = fileConfig0.get_optional<std::string>("pattern");
		if(level.has_value()) {
			setSinkLevel(File, spdlog::level::from_str(level.value()));
		}
		if(pattern.has_value()) {
			setSinkPattern(File, pattern.value());
		}
	}

	auto flushInterval = config.get_optional<int64_t>("flush-interval");
	if(flushInterval.has_value()) {
		spdlog::flush_every(std::chrono::seconds(flushInterval.value()));
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