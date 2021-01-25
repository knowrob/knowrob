:- module(lang_pragma, []).

:- use_module(library('lang/compiler')).

%% register query commands
:- query_compiler:add_command(pragma, [ask,tell]).

%%
% pragma(Goal) is evaluated compile-time by calling
% the Goal. This is usually done to unify variables
% used in the aggregation pipeline from the compile context.
%
query_compiler:step_compile(pragma(Goal), _, []) :-
	call(Goal).
