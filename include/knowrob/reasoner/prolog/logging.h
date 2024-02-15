/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_PROLOG_LOGGING_H_
#define KNOWROB_PROLOG_LOGGING_H_

#include <SWI-Prolog.h>

namespace knowrob::prolog {
	/**
	 * Implements the predicate `log_message/2`.
	 * @param level_term an atom encoding the log level
	 * @param msg_term the logged term
	 */
	foreign_t log_message2(term_t level_term, term_t msg_term);

	/**
	 * Implements the predicate `log_message/4`.
	 * @param level_term an atom encoding the log level
	 * @param msg_term the logged term
	 * @param file_term an atom encoding the file source
	 * @param line_term a number indicating the line number
	 */
	foreign_t log_message4(term_t level_term, term_t msg_term, term_t file_term, term_t line_term);
}

#endif // KNOWROB_PROLOG_LOGGING_H_
