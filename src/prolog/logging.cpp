/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <SWI-Prolog.h>
// gtest
#include <gtest/gtest.h>
// KnowRob
#include <knowrob/logging.h>
#include <knowrob/prolog/PrologQuery.h>
#include <knowrob/prolog/logging.h>

using namespace knowrob;

foreign_t pl_log_message(term_t arg0, term_t arg1)
{
	static atom_t ATOM_error         = PL_new_atom("error");
	static atom_t ATOM_warning       = PL_new_atom("warning");
	static atom_t ATOM_informational = PL_new_atom("informational");
	static atom_t ATOM_debug         = PL_new_atom("debug");

	// first argument is the log level
	atom_t level_atom;
	if(!PL_get_atom(arg0, &level_atom)) {
		return FALSE;
	}
	// second argument holds the logged term
	TermPtr logged_term = PrologQuery::constructTerm(arg1);

	// TODO: print line numbers in Prolog files
	if(level_atom == ATOM_debug) {
		KB_DEBUG("{}", *logged_term);
	}
	else if(level_atom == ATOM_informational) {
		KB_INFO("{}", *logged_term);
	}
	else if(level_atom == ATOM_warning) {
		KB_WARN("{}", *logged_term);
	}
	else if(level_atom == ATOM_error) {
		KB_ERROR("{}", *logged_term);
	}
	else {
		KB_WARN("unknown logging level '{}'.", PL_atom_chars(level_atom));
		KB_INFO("{}", *logged_term);
	}
	return TRUE;
}
