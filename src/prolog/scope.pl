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
	time: dict{
		min: dict{ max: Now },
		max: dict{ min: Now }
	}
}) :- get_time(Now).

%% time_scope(?Since,?Until,?Scope) is semidet.
%
% @param Scope A scope dict.
%
time_scope(Since, Until, Scope) :-
    var(Scope),!,
	Scope=dict{ time:
	    dict{ min: MinRange, max: MaxRange } },
	min_range(Since, MinRange),
	max_range(Until, MaxRange),
	!.

%%
min_range(=(Since),  dict{ min: Since, max: Since }).
min_range(>(Since),  dict{ min: Since }).
min_range(>=(Since), dict{ min: Since }).
min_range(<(Since),  dict{ max: Since }).
min_range(=<(Since), dict{ max: Since }).
min_range(Since,     dict{ min: Since, max: Since }).

%%
max_range(=(Until),  dict{ min: Until, max: Until }).
max_range(>(Until),  dict{ min: Until }).
max_range(>=(Until), dict{ min: Until }).
max_range(<(Until),  dict{ max: Until }).
max_range(=<(Until), dict{ max: Until }).
max_range(Until,     dict{ min: Until, max: Until }).

		 /*******************************
		 *	    UNIT TESTS	     		*
		 *******************************/

:- begin_tests(lang_scope).

test('scope_data([3,20])') :-
	time_scope(=(3), =(20), S),
	assert_unifies(S, _{ time: _{
	    min: _{ min: 3,  max: 3 },
	    max: _{ min: 20, max: 20 }
	}}).

test('scope_data([3,>=20])') :-
	time_scope(=(3), >=(20), S),
	assert_unifies(S, _{ time: _{
	    min: _{ min: 3,  max: 3 },
	    max: _{ min: 20 }
	}}).

test('scope_data([=<3,20])') :-
	time_scope(=<(3), =(20), S),
	assert_unifies(S, _{ time: _{
	    min: _{ max: 3 },
	    max: _{ min: 20, max: 20 }
	}}).

:- end_tests(lang_scope).
