/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef __KNOWROB_PROLOG_LOGGING_H__
#define __KNOWROB_PROLOG_LOGGING_H__

#include <SWI-Prolog.h>

foreign_t pl_log_message(term_t arg0, term_t arg1);

#endif // __KNOWROB_PROLOG_LOGGING_H__
