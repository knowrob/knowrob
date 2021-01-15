:- module(mng_scope,
    [ scope_step/2
    ]).

%% scope_step(+Context, -Step) is nondet.
%
%
%
scope_step(Context, Step) :-
	(	scope_intersect_(Context, Step)
	% skip documents with empty scope
	;	Step = ['$match', ['$expr', ['$lt', array([
				string('$v_scope.time.since'),
				string('$v_scope.time.until')])]]]
	).

%% 
scope_intersect_(Context,
		['$set', ['v_scope', Doc]]) :-
	% intersect old and new scope
	TimeScope = ['time', [
		['since', ['$max', array([string('$v_scope.time.since'),
		                          string('$next.scope.time.since')])]],
		['until', ['$min', array([string('$v_scope.time.until'),
		                          string('$next.scope.time.until')])]]
	]],
	(	memberchk(ignore,Context)
	->	Doc = ['$cond', array([
			['$not', array([string('$next.scope')]) ],
			string('$v_scope'),
			TimeScope
		])]
	;	Doc = TimeScope
	).
