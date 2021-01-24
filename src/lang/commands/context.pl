:- module(context_commands, []).

:- use_module(library('lang/compiler')).

%% register query commands
:- query_compiler:add_command(context, [ask,tell]).

%%
% context(-Option) and context(-Option, +Default) are used to read
% options from compile context to make them accessible in rules.
% The main usecase is that some temporal predicates need to access
% the query scope.
%
query_compiler:step_compile(
		context(Option),
		Context,
		[]) :-
	option(Option, Context).

query_compiler:step_compile(
		context(Option, Default),
		Context,
		[]) :-
	option(Option, Context, Default).
