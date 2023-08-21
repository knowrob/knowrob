:- module(lang_scope,
	[ query_scope_now/1,
	  time_scope/3
    ]).
/** <module> Scoping predicates.

@author Daniel Beßler
@license BSD
*/

%% query_scope_now(-Scope) is det.
%
% The scope of facts that are currently true.
%
% @param Scope A scope dictionary.
%
query_scope_now(dict{
	since: Now,
	until: Now
}) :- get_time(Now).

%% time_scope(?Since,?Until,?Scope) is semidet.
%
% @param Scope A scope dict.
%
time_scope(_, _, Scope) :-
    nonvar(Scope),!,
    fail.

time_scope(Since, Until, dict{
	since: Since,
	until: Until
}).
