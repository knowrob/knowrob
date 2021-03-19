:- module(swrl,
    [ implements('../ireasoner.pl'),
      swrl_assert/1
    ]).
/** <module> Prolog-based SWRL representation.

@author Daniel Beßler
*/

:- use_module(library('lang/terms/holds'),
    [ holds/3 ]).

:- dynamic swrl_instance_of/3,
           swrl_holds/4.
:- dynamic can_answer/1, holds_property_/2.

%%
% @implements ireasoner
%
can_answer(instance_of(_,_)).

%% Assert can_answer(holds(...)) for all super properties.
assert_can_answer_(Property) :-
  assert_can_answer1_(Property),
  assert_holds_property_(Property,Property),
  forall(transitive(subpropery_of(Property,Sup)), (
    assert_can_answer1_(Sup),
    assert_holds_property_(Property,Sup)
  )).

assert_can_answer1_(Property) :- can_answer(holds(_,Property,_)), !.
assert_can_answer1_(Property) :- assertz(can_answer(holds(_,Property,_))).

assert_holds_property_(P0,P1) :- holds_property_(P0,P1), !.
assert_holds_property_(P0,P1) :- assertz(holds_property_(P0,P1)).

%%
% @implements ireasoner
%
infer( holds(S,P,=(unit(Value,Unit))),
       holds(S,P,=(unit(Value,Unit))), QScope,FScope) :-
  !,
  % strip type
  % TODO: do not ignore data type
  % TODO: do not ignore data unit
  ( compound(Value) -> Value=..[_,Value1]; Value1=Value ),
  swrl_holds1(S,P,Value1,QScope->FScope).

infer( holds(S,P,O),
       holds(S,P,O), QScope,FScope) :-
  swrl_holds1(S,P,O,QScope->FScope),
  swrl_infer_scope(FScope).

infer( instance_of(S,O),
       instance_of(S,O), QScope,FScope) :-
  once((var(O);atom(O))),
  swrl_instance_of(S,O,QScope->FScope),
  swrl_infer_scope(FScope).

%%
swrl_infer_scope(FScope) :-
  nonvar(FScope).

swrl_infer_scope(FScope) :-
  var(FScope),
  universal_scope(FScope).

%%
swrl_holds1(S,P,O,Scope) :-
  var(P),!,
  swrl_holds(S,P,O,Scope).

swrl_holds1(S,P,O,Scope) :-
  holds_property_(PSub,P),!,
  swrl_holds(S,PSub,O,Scope).

%% swrl_assert(+Rule).
%
% Assert SWRL rule in the Prolog KB.
%
% @param Rule Prolog-based representation of SWRL rule.
%
swrl_assert([] :- _) :- !.

swrl_assert([HeadAtom|Xs] :- Body) :-
  swrl_assert1(HeadAtom :- Body),
  swrl_assert(Xs :- Body).

swrl_assert1(HeadAtom :- Body) :-
  swrl_vars([HeadAtom] :- Body, Vars),
  swrl_rule_pl(HeadAtom :- Body, Rule_pl, [var('swrl:scope',_)|Vars]),
  % assert additional can_answer clauses
  once(( Rule_pl=(:-(Implication,_)) ; Rule_pl=Implication )),
  ( Implication=swrl_holds(_,P,_,_) ->
    assert_can_answer_(P) ;
    true ),
  % finally assert swrl_holds or swrl_instance_of clause
  assertz( Rule_pl ).
  
swrl_rule_pl_implication(Implication :- _, Implication) :- !.
swrl_rule_pl_implication(Implication, Implication).

%% swrl_rule_pl
swrl_rule_pl(Fact :- [], Fact_pl, Vars) :-
  !,
  swrl_implication_pl(Fact, Fact_pl, Vars).

swrl_rule_pl(Impl :- Cond, Impl_pl :- Cond_pl, Vars) :-
  swrl_implication_pl(Impl, Impl_pl, Vars),
  swrl_condition_pl(Cond, Conditions, Vars),!,
  swrl_var(Vars, 'swrl:scope', Scope),
  Cond_pl=ask(Conditions,Scope).

%% swrl_implication_pl
swrl_implication_pl(class(Cls,S), swrl_instance_of(S_var,Cls,Scope), Vars) :-
  swrl_var(Vars, S, S_var),
  swrl_var(Vars, 'swrl:scope', Scope),!.

swrl_implication_pl(property(S,P,O), swrl_holds(S_var,P,O_var,Scope), Vars) :-
  swrl_var(Vars, S, S_var),
  swrl_var(Vars, O, O_var),
  swrl_var(Vars, 'swrl:scope', Scope),!.
  
%% swrl_condition_pl
swrl_condition_pl([], [], _) :- !.

swrl_condition_pl([X|Xs], [Y|Ys], Vars) :-
  swrl_condition_pl(X, Y, Vars),
  swrl_condition_pl(Xs, Ys, Vars).
  
swrl_condition_pl(class(Cls,S),    instance_of(S_var,Cls), Vars) :-
  swrl_var(Vars, S, S_var).

swrl_condition_pl(property(S,P,O), holds(S_var,P,O_var), Vars) :-
  swrl_var(Vars, S, S_var),
  swrl_var(Vars, O, O_var).

swrl_condition_pl(Builtin, { Builtin_pl }, Vars) :-
  swrl_builtin_pl(Builtin, Builtin_pl, Vars).

%% swrl_builtin_pl
% TODO (DB): support more builtins: matches, listConcat, member, length, ...
%
swrl_builtin_pl(equal(S,O), (S_var = O_var), Vars) :-
  swrl_var(Vars, S, S_var),
  swrl_var(Vars, O, O_var).

swrl_builtin_pl(notEqual(S,O), (S_var \= O_var), Vars) :-
  swrl_var(Vars, S, S_var),
  swrl_var(Vars, O, O_var).

swrl_builtin_pl(lessThan(A,B), swrl:swrl_less(A_num,B_num), Vars) :-
  swrl_nums([A,B], [A_num,B_num], Vars).

swrl_builtin_pl(lessThanOrEqual(A,B), swrl:swrl_leq(A_num,B_num), Vars) :-
  swrl_nums([A,B], [A_num,B_num], Vars).

swrl_builtin_pl(greaterThan(A,B), swrl:swrl_greater(A_num,B_num), Vars) :-
  swrl_nums([A,B], [A_num,B_num], Vars).

swrl_builtin_pl(greaterThanOrEqual(A,B), swrl:swrl_geq(A_num,B_num), Vars) :-
  swrl_nums([A,B], [A_num,B_num], Vars).

%% math builtins
swrl_builtin_pl(add(A,B,C), swrl:swrl_plus(A_num,B_num,C_num), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).

swrl_builtin_pl(subtract(A,B,C), swrl:swrl_minus(A_num,B_num,C_num), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).

swrl_builtin_pl(multiply(A,B,C), swrl:swrl_multiply(A_num,B_num,C_num), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).

swrl_builtin_pl(divide(A,B,C), swrl:swrl_divide(A_num,B_num,C_num), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).

swrl_builtin_pl(mod(A,B,C), swrl:swrl_mod(A_num,B_num,C_num), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).

swrl_builtin_pl(pow(A,B,C), swrl:swrl_pow(A_num,B_num,C_num), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).

swrl_builtin_pl(abs(A,B), swrl:swrl_abs(B_num,A_num), Vars) :-
  swrl_nums([A,B], [A_num,B_num], Vars).

%% boolean builtins
swrl_builtin_pl(booleanNot(S), not(S_var), Vars) :-
  swrl_var(Vars, S, S_var).

%% string builtins
swrl_builtin_pl(stringConcat(A,B,C), atom_concat(C_atom,A_atom,B_atom), Vars) :-
  swrl_atoms([A,B,C], [A_atom,B_atom,C_atom], Vars).

swrl_builtin_pl(stringLength(A,L), atom_length(A_atom, L_num), Vars) :-
  swrl_atom(A,A_atom,Vars),
  swrl_atom_number(L,L_num,Vars).

swrl_builtin_pl(upperCase(A,Upper), upcase_atom(A_atom, Upper_atom), Vars) :-
  swrl_atoms([A,Upper], [A_atom,Upper_atom], Vars).

swrl_builtin_pl(lowerCase(A,Lower), downcase_atom(A_atom, Lower_atom), Vars) :-
  swrl_atoms([A,Lower], [A_atom,Lower_atom], Vars).

swrl_builtin_pl(contains(A,X), sub_atom(A_atom, _, _, _, X_atom), Vars) :-
  swrl_atoms([A,X], [A_atom,X_atom], Vars).

swrl_builtin_pl(startsWith(A,X), atom_concat(X_atom, _, A_atom), Vars) :-
  swrl_atoms([A,X], [A_atom,X_atom], Vars).

swrl_builtin_pl(endsWith(A,X), atom_concat(_, X_atom, A_atom), Vars) :-
  swrl_atoms([A,X], [A_atom,X_atom], Vars).

%% extensions
swrl_builtin_pl(makeOWLClass(A), swrl:swrlx_make_individual(A_atom), Vars) :-
  swrl_atoms([A], [A_atom], Vars).

%%
swrl_leq(A,B)        :- number_list([A,B],[X,Y]),     X =< Y.
swrl_less(A,B)       :- number_list([A,B],[X,Y]),     X < Y.
swrl_geq(A,B)        :- number_list([A,B],[X,Y]),     X >= Y.
swrl_greater(A,B)    :- number_list([A,B],[X,Y]),     X > Y.
swrl_plus(A,B,C)     :- number_list([A,B,C],[X,Y,Z]), X is Y + Z.
swrl_minus(A,B,C)    :- number_list([A,B,C],[X,Y,Z]), X is Y - Z.
swrl_divide(A,B,C)   :- number_list([A,B,C],[X,Y,Z]), X is Y / Z.
swrl_multiply(A,B,C) :- number_list([A,B,C],[X,Y,Z]), X is Y * Z.
swrl_mod(A,B,C)      :- number_list([A,B,C],[X,Y,Z]), X is Y mod Z.
swrl_pow(A,B,C)      :- number_list([A,B,C],[X,Y,Z]), X is Y ** Z.
swrl_abs(A,B)        :- number_list([A,B],[X,Y]),     X is abs(Y).

%% number_list: enforce number type on list elements
number_list([],[]).
number_list([X|Xs],[X|Ys]) :-
  var(X), !, number_list(Xs,Ys).
number_list([X|Xs],[X|Ys]) :-
  number(X), !, number_list(Xs,Ys).
number_list([X|Xs],[Y|Ys]) :-
  atom(X), !,
  catch(atom_number(X,Y), _, fail),
  number_list(Xs,Ys).
number_list([X|Xs],[Y|Ys]) :-
  catch(atom_number(X,Y), _, fail),
  number_list(Xs,Ys).

%% Find all the variables in a SWRL rule and map those to anonymous
%% Prolog variables for later unification.
swrl_vars(Head :- Body, Variables) :-
  findall(VarName, (
    (member(Atom,Head) ; member(Atom,Body)),
    swrl_var_names(Atom,VarName)
  ), VarNames),
  list_to_set(VarNames, VarNamesUnique),
  maplist(swrl_var_pl(_), VarNamesUnique,Variables).
swrl_var_pl(_,Y,var(Y,_)).

%% Find variables in a SWRL atom
swrl_var_names(var(Var), Var) :- atom(Var), !.
swrl_var_names(Term, Var) :-
  compound(Term), !,
  Term =.. [_|Args],
  member(Arg, Args),
  swrl_var_names(Arg, Var).

%% Map between name of a SWRL variable and the corresponding
%% Prolog variable.
swrl_var(Vars, Name, PrologVar) :-
  atom(Name), member(var(Name,PrologVar), Vars), !.
swrl_var(Vars, var(X), PrologVar) :-
  swrl_var(Vars, X, PrologVar), !.
swrl_var(_, Name, Name).

%%
set_vars(_, []).
set_vars(Vars, [var(Name,Var)|Rest]) :-
  swrl_var(Vars, Name, Var),
  set_vars(Vars, Rest).

%%
swrl_vars_resolve([], [], _) :- !.
swrl_vars_resolve([var(X)|Xs], [Y|Ys], Vars) :- !,
  swrl_var(Vars, X, Y),
  swrl_vars_resolve(Xs, Ys, Vars).
swrl_vars_resolve([X|Xs], [X|Ys], Vars) :-
  swrl_vars_resolve(Xs, Ys, Vars).

%%
swrl_atom(A, B, _) :-
  var(A),!, B=A.
swrl_atom(var(A), A_val, Vars) :-
  swrl_var(Vars, A, A_var),
  swrl_atom(A_var,A_val,Vars).
swrl_atom(Atom, Atom, _) :-
  atom(Atom).
swrl_atoms([], [], _).
swrl_atoms([X|Xs], [Y|Ys], Vars) :-
  swrl_atom(X,Y,Vars),
  swrl_atoms(Xs,Ys,Vars).

%%
swrl_atom_number(var(A), A_num, Vars) :-
  !, swrl_var(Vars, A, A_var),
  ( ground(A_var) ->
    swrl_atom_number(A_var, A_num, Vars);
    A_num = A_var
  ).

swrl_atom_number(A, A_num, _) :-
  atom(A), !,
  catch(atom_number(A,A_num), _, fail).

swrl_atom_number(A_num, A_num, _) :-
  number(A_num), !.

%%
swrl_nums([],[],_).
swrl_nums([X|Xs],[Y|Ys],Vars) :-
  swrl_atom_number(X,Y,Vars),
  swrl_nums(Xs,Ys,Vars).

%% swrlx
swrlx_make_individual(A) :-
  ground(A).

swrlx_make_individual(A) :-
  \+ ground(A),
  tell(instance_of(A, owl:'Thing')).
