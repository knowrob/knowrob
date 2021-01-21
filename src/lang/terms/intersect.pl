:- module(lang_intersect, []).

:- use_module(library('db/mongo/compiler')).
:- use_module(library('db/mongo/query')).

%% register query commands
:- add_query_command(intersect).

%%
query_compiler:step_var(intersect(Scope), Var) :-
	time_scope(Since, Until, Scope),
	member(X, [Since,Until]),
	mng_strip(X, '=', _Type, Y),
	query_compiler:step_var(Y, Var).

%%
query_compiler:step_compile(
		intersect(Scope), _Context,
		[	['$set', ['v_scope', ['time', [
				['since', ['$max', array([string('$v_scope.time.since'),Since0])]],
				['until', ['$min', array([string('$v_scope.time.until'),Until0])]]
			]]]],
			ValidationStep
		]) :-
	% get since/until values
	time_scope(Since, Until, Scope),
	query_compiler:var_key_or_val(Since,Since0),
	query_compiler:var_key_or_val(Until,Until0),
	% ensure the scope is not empty after intersection
	scope_validation(ValidationStep).
	