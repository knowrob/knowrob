#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

/*
static PL_extension predicates[] = {
{ "foo",        1,      pl_foo, 0 },
{ "bar",        2,      pl_bar, PL_FA_NONDETERMINISTIC },
{ NULL,         0,      NULL,   0 }
};

main(int argc, char **argv)
{ PL_register_extensions_in_module("user", predicates);

  if ( !PL_initialise(argc, argv) )
    PL_halt(1);

  ...
}

 static foreign_t
atom_checksum(term_t a0, int arity, void* context)
{ char *s;

  if ( PL_get_atom_chars(a0, &s) )
  { int sum;

    for(sum=0; *s; s++)
      sum += *s&0xff;

    return PL_unify_integer(a0+1, sum&0xff);
  }

  return FALSE;
}

install_t
install()
{ PL_register_foreign("atom_checksum", 2,
                      atom_checksum, PL_FA_VARARGS);
}
 */

// +Goal, +Options
PREDICATE(kb_call1, 2) {
	/*
kb_call1(SubGoals, Options) :-
	% create a list of step(SubGoal, OutQueue, Channels) terms
	maplist([SubGoal,Step]>>
		query_step(SubGoal,Step),
		SubGoals, Steps),
	% combine steps if possible
	combine_steps(Steps, Combined),
	% need to remember pattern of variables for later unification
	term_variables(SubGoals, Pattern),
	setup_call_cleanup(
		start_pipeline(Combined, Pattern, Options, FinalStep),
		materialize_pipeline(FinalStep, Pattern, Options),
		stop_pipeline(Combined)
	).
	 */
	return TRUE;
}

// +Goal, +Options
PREDICATE(kb_project1, 1) {
	return TRUE;
}

// +Goal, +Scope, +Options
PREDICATE(kb_unproject, 3) {
	return TRUE;
}

PREDICATE(memorize, 3) {
	/*

%% memorize(+Directory) is det.
%
% Store knowledge into given directory.
%
% @param Directory filesystem path
%
memorize(Directory) :-
	mng_export(Directory).
	 */
	return TRUE;
}

PREDICATE(remember, 3) {
	/*
%% remember(+Directory) is det.
%
% Restore memory previously stored into given directory.
%
% @param Directory filesystem path
%
remember(Directory) :-
	mng_import(Directory).
	 */
	return TRUE;
}
