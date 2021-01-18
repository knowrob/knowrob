:- module(mng_term_pragma,
    [ match_operator/2
    ]).

:- use_module(library('db/mongo/lang/compiler')).
:- use_module(library('db/mongo/lang/query')).

%% register query commands
:- mng_query_command(pragma).

%%
% pragma(Goal) is evaluated compile-time by calling
% the Goal. This is usually done to unify variables
% used in the aggregation pipeline from the call context.
%
mng_compiler:step_compile(
		pragma(Goal),
		_Context, []) :-
	call(Goal).

