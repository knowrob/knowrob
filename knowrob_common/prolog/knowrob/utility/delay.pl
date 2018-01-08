% as downloaded from http://www.swi-prolog.org/pack/file_details/delay/prolog/delay.pl?show=src
:- module(delay, [ delay/1
                 , univ/3
                 , when_proper_list/2
                 ]).
:- use_module(library(when), [when/2]).

:- multifile mode/1.
mode('$dcg':phrase(nonvar,ground)).
mode('$dcg':phrase(ground,_)).

mode('$dcg':phrase(nonvar,ground,_)).
mode('$dcg':phrase(ground,_,_)).

mode(apply:maplist(nonvar,list,_)).
mode(apply:maplist(nonvar,_,list)).

mode(delay:univ(nonvar,_,_)).
mode(delay:univ(_,ground,list)).

mode(lists:reverse(list,_)).
mode(lists:reverse(_,list)).

mode(lists:same_length(list,_)).
mode(lists:same_length(_,list)).

mode(system:atom(nonvar)).

mode(system:atom_codes(ground, _)).
mode(system:atom_codes(_, ground)).

mode(system:atomic_list_concat(ground,ground,_)).
mode(system:atomic_list_concat(_,ground,ground)).

mode(system:functor(nonvar,_,_)).
mode(system:functor(_,ground,ground)).

mode(system:float(nonvar)).

mode(system:dict_pairs(nonvar,_,_)).
mode(system:dict_pairs(_,_,list)).

mode(system:integer(nonvar)).

mode(system:is_dict(nonvar)).

mode(system:is_dict(nonvar,_)).

mode(system:length(_,ground)).
mode(system:length(list,_)).

mode(system:number_codes(ground,_)).
mode(system:number_codes(_,ground)).

mode(system:plus(ground,ground,_)).
mode(system:plus(ground,_,ground)).
mode(system:plus(_,ground,ground)).

mode(system:string_codes(ground,_)).
mode(system:string_codes(_,ground)).

mode(system:succ(ground,_)).
mode(system:succ(_,ground)).


%%	univ(+Term, -Name, -Args) is det.
%%	univ(-Term, +Name, +Args) is det.
%
%   Just like is like `Term =.. [Name|Args]`. This predicate is exported
%   to placate the cross-referencer. It's intended
%   to be called as `delay(univ(T,N,As))`. Although it can be used as a
%   normal goal, if wanted.
univ(Term, Name, Args) :-
    Term =.. [Name|Args].


%%	delay(:Goal)
%
%   Like `call(Goal)` but postpones execution until Goal's arguments are
%   bound enough to avoid errors like: "Arguments are not sufficiently
%   instantiated". This is currently realized with attributed
%   variables and when/2, so execution timing is identical. For example,
%
%       t :-
%           delay(atom_codes(A,C)),
%           A = hello,
%           C == "hello".
%
%   does not throw an exception on the first line.
%   One is simply declaring that `A` and `C` have a given relationship
%   without stating when the predicate (atom_codes/2) will
%   execute. This declarative style is especially valuable when
%   different modes of a predicate require different goal order.
%
%   The following predicates are currently supported:
%
%     * atom_codes/2
%     * functor/3
%     * length/2
%     * number_codes/2
%     * phrase/2
%     * phrase/3
%     * plus/3
%     * succ/2
%     * univ/3
%
%   `delay(length(L,Len))` warrants additional explanation. length/2
%   doesn't throw instantiation exceptions. It simply iterates all
%   possible lists and their respective lengths. This isn't always
%   ideal. Using delay/1 with length/2 yields the same semantics but
%   performs much less backtracking.  It waits until either `L`
%   or `Len` is bound then length/2 evaluates without any choicepoints.
%   `L` must become a proper list to trigger, so incrementally binding
%   its head is OK.
:- dynamic delay/1, delay_followup/1.
:- meta_predicate delay(0).
delay(Module:Goal) :-
    % build a delay/1 clause to support Goal

    goal_to_conditions(Module:Goal, Head, SimpleConditions, ComplexConditions),
    !,
    ( SimpleConditions==ComplexConditions ->
        DelayedGoal = Module:Head
    ; % otherwise ->
        DelayedGoal = delay_followup(Module:Head),
        maplist(assert_followup_clause(Module:Head), ComplexConditions)
    ),

    maplist(xfy_list(','), Simples, SimpleConditions),
    xfy_list(';', Condition, Simples),
    asserta((
        delay(Module:Head) :-
            !,
            when(Condition, DelayedGoal)
    )),
    delay(Module:Goal).
delay(Module:Goal) :-
    functor(Goal, Name, Arity),
    format(atom(Msg), '~w:~w/~d not supported. See delay:mode/1', [Module,Name,Arity]),
    throw(Msg).


% like this:
% goal_to_conditions(
%     length(L,Len),
%     length(X,Y),
%     [[nonvar(X)],[ground(Y)]],
%     [[list(X)],[ground(Y)]]
% )
goal_to_conditions(Module:Goal, Head, SimpleConditions, ComplexConditions) :-
    functor(Goal, Name, Arity),
    functor(Head, Name, Arity),
    Head =.. [Name|HeadArgs],

    % find all modes for this goal
    ( setof(Head, mode(Module:Head), Modes) ->
        true
    ; predicate_property(Module:Head, imported_from(Origin)) ->
        setof(Head, mode(Origin:Head), Modes)
    ),

    partition_modes(Modes, HeadArgs, SimpleConditions, ComplexConditions).


partition_modes([], _, [], []).
partition_modes([Mode|Modes], HeadArgs, [SimpleH|SimpleT], [ComplexH|ComplexT]) :-
    Mode =.. [_|ModeArgs],
    map_include(make_condition, ModeArgs, HeadArgs, SimpleH, ComplexH),
    partition_modes(Modes, HeadArgs, SimpleT, ComplexT).


% convert a mode name and argument variable into when/2 conditions
make_condition(X, _, _, _) :-
    var(X),
    !,
    fail.
make_condition(ground, X, ground(X), ground(X)).
make_condition(nonvar, X, nonvar(X), nonvar(X)).
make_condition(list, X, nonvar(X), list(X)).


% create an additional clause for delay_followup/1
assert_followup_clause(Module:Head, ComplexConditions) :-
    exclude(is_list_mode, ComplexConditions, GuardConditions),
    include(is_list_mode, ComplexConditions, ListConditions),
    ( ListConditions=[] ->
        xfy_list(',', Guard, GuardConditions),
        Goal = Module:Head
    ; ListConditions=[list(List)] ->
        xfy_list(',', Guard, [nonvar(List)|GuardConditions]),
        Goal = when_proper_list(List, Module:Head)
    ; % otherwise ->
        throw('Predicates with multiple `list` modes are not supported')
    ),

    assertz((
        delay_followup(Module:Head) :-
            Guard,
            !,
            Goal
    )).


% true if the given mode represents 'list'
is_list_mode(list(_)).


%%	when_proper_list(List, Goal)
%
%   Delay executing Goal until List becomes a proper list. This
%   predicate is part of the internal implementation of delay/1 but it
%   may be useful to others so it's exported.
:- meta_predicate when_proper_list(?,0).
when_proper_list(List, Goal) :-
    var(List),
    !,
    when(nonvar(List), when_proper_list(List, Goal)).
when_proper_list([], Goal) :-
    call(Goal).
when_proper_list([_|T], Goal) :-
    when_proper_list(T, Goal).


% like maplist but skips elements for which Goal fails.
% it's like the love child of maplist and exclude.
:- meta_predicate map_include(4, +, +, -, -).
:- meta_predicate map_include_(+,+,4,-,-).
map_include(F, La, Lb, Lc, Ld) :-
    map_include_(La, Lb, F, Lc, Ld).
map_include_([], [], _, [], []).
map_include_([Ha|Ta], [Hb|Tb], F, Lc0, Ld0) :-
    ( call(F, Ha, Hb, Hc, Hd) ->
        Lc0 = [Hc|Lc],
        Ld0 = [Hd|Ld]
    ; % otherwise ->
        Lc0 = Lc,
        Ld0 = Ld
    ),
    map_include_(Ta, Tb, F, Lc, Ld).


% originally copied from library(list_util).
% I don't want this pack to depend on external libraries.
xfy_list(Op, Term, [Left|List]) :-
    Term =.. [Op, Left, Right],
    xfy_list(Op, Right, List),
    !.
xfy_list(_, Term, [Term]).
