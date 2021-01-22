:- module(lang_intersect, []).

:- use_module(library('lang/compiler')).

%% register query commands
:- query_command_add(intersect).

%%
query_compiler:step_var(intersect(Scope), Var) :-
	Scope=_{ time: _{ since: Since, until: Until }},
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
	Scope=_{ time: _{ since: Since, until: Until }},
	query_compiler:var_key_or_val(Since,Since0),
	query_compiler:var_key_or_val(Until,Until0),
	% ensure the scope is not empty after intersection
	scope_validation(ValidationStep).
	