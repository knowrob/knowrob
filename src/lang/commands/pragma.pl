:- module(lang_pragma,
    [ match_operator/2
    ]).

:- use_module(library('lang/compiler')).

%% register query commands
:- query_compiler:add_command(pragma, [ask,tell]).

%%
% pragma(Goal) is evaluated compile-time by calling
% the Goal. This is usually done to unify variables
% used in the aggregation pipeline from the call context.
%
query_compiler:step_compile(
		pragma(Goal), _Context, []) :-
	call(Goal).
