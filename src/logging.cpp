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

#define LOGFILE_MAX_SIZE (1024*1024)
#define LOGFILE_MAX_FILES 4
#define LOGFILE_ROTATE_ON_OPEN true

void knowrob::logging::initialize() {
	auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
	console_sink->set_level(spdlog::level::info);
	console_sink->set_pattern("[%$%H:%M:%S.%e] [%^%l%$] %v");
	
	// When the max file size is reached, close the file, rename it, and create a new file.
	// Both the max file size and the max number of files are configurable in the constructor.
	auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
		"logs/rotating.txt",
		LOGFILE_MAX_SIZE,
		LOGFILE_MAX_FILES,
		LOGFILE_ROTATE_ON_OPEN);
	file_sink->set_level(spdlog::level::trace);
	file_sink->set_pattern("[%c] [thread:%t] [%^%l%$] %v (%s:%#)");
	
	spdlog::set_default_logger(std::make_shared<spdlog::logger>("multi_sink",
		spdlog::sinks_init_list({console_sink, file_sink})));
	spdlog::set_level(spdlog::level::trace);
}
