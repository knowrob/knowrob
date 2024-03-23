/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <SWI-Prolog.h>
#include "knowrob/Logger.h"
#include "knowrob/reasoner/prolog/PrologTerm.h"
#include "knowrob/reasoner/prolog/logging.h"

using namespace knowrob;

static inline foreign_t pl_log_message_internal(term_t level_term, term_t msg_term, const char *file, int line) {
	static atom_t ATOM_error = PL_new_atom("error");
	static atom_t ATOM_warning = PL_new_atom("warning");
	static atom_t ATOM_informational = PL_new_atom("informational");
	static atom_t ATOM_debug = PL_new_atom("debug");

	// first argument is the log level
	atom_t level_atom;
	if (!PL_get_atom(level_term, &level_atom)) {
		return FALSE;
	}
	// second argument holds the logged term
	TermPtr msg = PrologTerm::toKnowRobTerm(msg_term);

	// remove single quotes from logged message which are added
	// by the string conversion of Term objects.
	std::stringstream ss;
	if (msg->isAtomic()) {
		ss << std::static_pointer_cast<Atomic>(msg)->stringForm();
	}
	std::string msg_str = ss.str();
	if (msg_str.front() == '\'') {
		msg_str.erase(0, 1);
		msg_str.erase(msg_str.size() - 1);
	}

	// finally call the logging macro according to the level
	// of the log message
	if (level_atom == ATOM_debug) {
		KB_DEBUG1(file, line, "{}", msg_str);
	} else if (level_atom == ATOM_informational) {
		KB_INFO1(file, line, "{}", msg_str);
	} else if (level_atom == ATOM_warning) {
		KB_WARN1(file, line, "{}", msg_str);
	} else if (level_atom == ATOM_error) {
		KB_ERROR1(file, line, "{}", msg_str);
	} else {
		KB_WARN("unknown logging level '{}'.", PL_atom_chars(level_atom));
		KB_INFO1(file, line, "{}", *msg);
	}
	return TRUE;
}

namespace knowrob::prolog {
	foreign_t log_message2(term_t level_term, term_t msg_term) {
		return pl_log_message_internal(level_term, msg_term, __FILE__, __LINE__);
	}

	foreign_t log_message4(term_t level_term, term_t msg_term, term_t file_term, term_t line_term) {
		atom_t file_atom;
		int line_number = 0;
		if (PL_get_atom(file_term, &file_atom) &&
			PL_get_integer(line_term, &line_number)) {
			const char *file_string = PL_atom_chars(file_atom);
			return pl_log_message_internal(level_term, msg_term, file_string, line_number);
		} else {
			return log_message2(level_term, msg_term);
		}
	}
}
