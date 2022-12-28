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

/**
 * Implements the predicate `log_message/2`.
 * @param level_term an atom encoding the log level
 * @param msg_term the logged term
 */
foreign_t pl_log_message2(term_t level_term, term_t msg_term);

/**
 * Implements the predicate `log_message/4`.
 * @param level_term an atom encoding the log level
 * @param msg_term the logged term
 * @param file_term an atom encoding the file source
 * @param line_term a number indicating the line number
 */
foreign_t pl_log_message4(term_t level_term, term_t msg_term, term_t file_term, term_t line_term);

#endif // __KNOWROB_PROLOG_LOGGING_H__
