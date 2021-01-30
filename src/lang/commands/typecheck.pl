:- module(typecheck_commands, []).

:- use_module(library('lang/compiler')).

%% register query commands
:- query_compiler:add_command(ground,  [ask,tell]).
:- query_compiler:add_command(var,     [ask,tell]).
:- query_compiler:add_command(number,  [ask,tell]).
:- query_compiler:add_command(atom,    [ask,tell]).
:- query_compiler:add_command(is_list, [ask,tell]).

%%
query_compiler:step_var(ground(X),  [V,X]) :- query_compiler:var_key(X, V).
query_compiler:step_var(var(X),     [V,X]) :- query_compiler:var_key(X, V).
query_compiler:step_var(number(X),  [V,X]) :- query_compiler:var_key(X, V).
query_compiler:step_var(atom(X),    [V,X]) :- query_compiler:var_key(X, V).
query_compiler:step_var(is_list(X), [V,X]) :- query_compiler:var_key(X, V).

%%
% ground(X) holds iff a previous step has assigned
% a value to the variable key.
%
query_compiler:step_compile(ground(Arg), _Context, []) :-
	% is grounded already compile-time
	ground(Arg), !.

query_compiler:step_compile(
		ground(Arg), _Context,
		[['$match', [
			[Key, ['$exists', bool(true)]]
		]]]) :-
	query_compiler:var_key(Arg, Key).

%%
% var(X) holds iff no previous step has assigned
% a value to the variable key.
%
query_compiler:step_compile(var(Arg), _Context, []) :-
	% is grounded already compile-time
	% TODO: if var(Arg) and the arg was not referred before, succeed without this step
	ground(Arg), !,
	fail.

query_compiler:step_compile(
		var(Arg), _Context,
		[['$match', [
			[Key, ['$exists', bool(false)]]
		]]]) :-
	query_compiler:var_key(Arg, Key).

%%
%
%
query_compiler:step_compile(number(Arg), _Context, []) :-
	% is grounded already compile-time
	ground(Arg), !,
	number(Arg).

% NOTE: mongo DB 4.4 has $isNumber command
query_compiler:step_compile(
		number(Arg), _Context,
		[['$match', ['$or', array([
			[Key, ['$type', int(1)]],
			[Key, ['$type', int(16)]],
			[Key, ['$type', int(18)]]
		])]]]) :-
	% TODO: if not ground in the call and not referred to before,
	%        this step can be skipped and step_compile should fail then.
	query_compiler:var_key(Arg, Key).

%%
%
%
query_compiler:step_compile(
		atom(Arg), _Context, Pipeline) :-
	match_type_(Arg, atom, string, Pipeline).

%%
%
%
query_compiler:step_compile(
		is_list(Arg), _Context, Pipeline) :-
	match_type_(Arg, is_list, array, Pipeline).


%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% helper
%%%%%%%%%%%%%%%%%%%%%%%

%%
match_type_(Arg, Goal, _Type, []) :-
	% argument is grounded already compile-time
	ground(Arg), !,
	call(Goal, [Arg]).

match_type_(Arg, _Goal, Type, 
		[['$match',
			[Key, ['$type', string(Type)]]
		]]) :-
	% TODO: if not ground in the call and not referred to before,
	%        this step can be skipped and step_compile should fail then.
	query_compiler:var_key(Arg, Key).
