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

namespace knowrob::logging {
	void initialize();
}

#endif //__KNOWROB_LOGGING_H__

