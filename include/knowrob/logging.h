/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_LOGGING_H__
#define __KNOWROB_LOGGING_H__

// note: must be defined before including spdlog.h
//#ifdef _DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
//#else
//#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO
//#endif

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

#define KB_TRACE    SPDLOG_TRACE
#define KB_DEBUG    SPDLOG_DEBUG
#define KB_INFO     SPDLOG_INFO
#define KB_WARN     SPDLOG_WARN
#define KB_ERROR    SPDLOG_ERROR
#define KB_CRITICAL SPDLOG_CRITICAL

#define KB_LOGGER_CALL(file, line, level, ...) \
	spdlog::default_logger_raw()->log(spdlog::source_loc{file, line, SPDLOG_FUNCTION}, level, __VA_ARGS__)

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_TRACE
#    define KB_TRACE1(file, line, ...) KB_LOGGER_CALL(file, line, spdlog::level::trace, __VA_ARGS__)
#else
#    define KB_TRACE(file, line, ...) (void)0
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_DEBUG
#    define KB_DEBUG1(file, line, ...) KB_LOGGER_CALL(file, line, spdlog::level::debug, __VA_ARGS__)
#else
#    define KB_DEBUG(file, line, ...) (void)0
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_INFO
#    define KB_INFO1(file, line, ...) KB_LOGGER_CALL(file, line, spdlog::level::info, __VA_ARGS__)
#else
#    define KB_INFO(file, line, ...) (void)0
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_WARN
#    define KB_WARN1(file, line, ...) KB_LOGGER_CALL(file, line, spdlog::level::warn, __VA_ARGS__)
#else
#    define KB_WARN(file, line, ...) (void)0
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_ERROR
#    define KB_ERROR1(file, line, ...) KB_LOGGER_CALL(file, line, spdlog::level::err, __VA_ARGS__)
#else
#    define KB_ERROR(file, line, ...) (void)0
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_CRITICAL
#    define KB_CRITICAL1(file, line, ...) KB_LOGGER_CALL(file, line, spdlog::level::critical, __VA_ARGS__)
#else
#    define KB_CRITICAL(file, line, ...) (void)0
#endif

namespace knowrob::logging {
	void initialize();
}

#endif //__KNOWROB_LOGGING_H__

