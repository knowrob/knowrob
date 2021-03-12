:- module(lang_intersect,
		[ mng_scope_intersect/5 ]).

:- use_module(library('lang/scope'),
		[ time_scope/3 ]).
:- use_module(library('lang/mongolog/mongolog')).

%% mng_scope_intersect(+VarKey, +Since1, +Until1, +Options, -Step) is nondet.
%
% The step expects input documents with VarKey
% field, and another field Since1/Until1.
% The step uses these field to compute an intersection
% beteween both scopes.
% It will fail in case the intersection is empty.
%
mng_scope_intersect(VarKey, Since1, Until1, Options, Step) :-
	atomic_list_concat(['$',VarKey,'.time.since'], '', Since0),
	atomic_list_concat(['$',VarKey,'.time.until'], '', Until0),
	atomic_list_concat(['$',VarKey], '', VarKey0),
	%
	Intersect = ['time', [
		['since', ['$max', array([string(Since0), Since1])]],
		['until', ['$min', array([string(Until0), Until1])]]
	]],
	% check if ignore flag if set, if so use a conditional step
	(	memberchk(ignore, Options)
	->	IntersectStep = ['$cond', array([
			['$not', array([Since1])],
			string(VarKey0),
			Intersect
		])]
	;	IntersectStep = Intersect
	),
	% first compute the intersection
	(	Step=['$set', ['v_scope', IntersectStep]]
	% then verify that the scope is non empty
	;	Step=['$match', ['$expr',
			['$lt', array([string(Since0), string(Until0)])]
		]]
	).

	