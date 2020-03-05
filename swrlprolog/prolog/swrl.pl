
:- module(swrl,
    [
      swrl_assert/1,
      swrl_project/1,
      swrl_project/2,
      swrl_satisfied/1,
      swrl_satisfied/2
    ]).
/** <module> Prolog-based SWRL representation.

@author Daniel Beßler
*/

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
%% TODO: remove knowrob_common dependency.
%%         but SWRL needs to assert triples,
%%         and use the OWL+SWRL reasoner in conditions.
%%         it would be cool if we could introduce a reasoner in a more
%%         general way.
:- use_module(library('knowrob/knowrob')).

:- rdf_db:rdf_register_prefix(swrl, 'http://www.w3.org/2003/11/swrl#', [keep(true)]).

:- rdf_meta swrl_project(r),
            swrl_match_instance(r,r,r).
:- dynamic  call_mutex/2.

%%
knowrob:vkb_has_triple(S,P,O,DBArgs) :-
  rdf_has(swrl_registry,P,literal(Command)),
  call(Command,S,O,DBArgs).
%%
swrl_assert_triple(S,P,O) :-
  kb_assert(S,P,O).
%%
swrl_triple(S,P,O,DBArgs) :-
  kb_triple(S,P,O,DBArgs).
%%
swrl_type_of(S,Cls,DBArgs) :-
  kb_type_of(S,Cls,DBArgs).


%% swrl_assert(+Rule).
%
% Assert SWRL rule in the Prolog KB.
%
% @param Rule Prolog-based representation of SWRL rule.
%
swrl_assert(Rule) :- swrl_assert(Rule,_).

%% swrl_assert(+Rule, -Rule_pl).
%
% Assert SWRL rule in the Prolog KB.
%
% @param Rule Prolog-based representation of SWRL rule.
% @param Rule_pl Prolog rule that corresponds to a SWRL rule.
%
swrl_assert([] :- _, []) :- !.
swrl_assert([HeadAtom|Xs] :- Body, [Asserted|Rest]) :-
  swrl_vars([HeadAtom] :- Body, Vars),
  swrl_rule_pl(HeadAtom :- Body, Rule_pl, [var('swrl:dbargs',_)|Vars]),
  swrl_rule_pl_implication(Rule_pl, swrl_holds(S_X,P_X,O_X,I_X)),
  % assert prolog rule
  rdf_split_url(_, P_name, P_X),
  atom_concat('swrl_comp_', P_name, PredicateName),
  Predicate=..[PredicateName,S_X,O_X,I_X],
  ( Rule_pl=(_:-Condition) ->
    Asserted=(Predicate:-Condition);
    Asserted=Predicate ),
  assertz( Asserted ),
  once((
    rdf_has(swrl_registry,P_X,_);
    rdf_assert(swrl_registry,P_X,literal(PredicateName),swrl)
  )),
  %%
  swrl_assert(Xs :- Body, Rest).
  
swrl_rule_pl_implication(Implication :- _, Implication) :- !.
swrl_rule_pl_implication(Implication, Implication).

%% swrl_rule_pl
swrl_rule_pl(Fact :- [], Fact_pl, Vars) :-
  !, swrl_implication_pl(Fact, Fact_pl, Vars).
swrl_rule_pl(Impl :- Cond, Impl_pl :- swrl:with_call_mutex(MutexId, Cond_pl), Vars) :-
  swrl_implication_pl(Impl, Impl_pl, Vars),
  swrl_condition_pl(Cond, Cond_pl, Vars),!,
  % FIXME not nice
  random(0, 999999999, MutexId).

%% avoid that rules call themself in conditions
with_call_mutex(MutexId, Goal) :-
  thread_self(ThreadId),
  \+ call_mutex(MutexId,ThreadId),
  setup_call_cleanup(
    assert(call_mutex(MutexId,ThreadId)),
    Goal,
    retract(call_mutex(MutexId,ThreadId))
  ).

%% swrl_implication_pl
swrl_implication_pl(class(Cls,S), swrl_holds(S_var,P,Cls_rdf,DBArgs), Vars) :-
  swrl_var(Vars, S, S_var),
  swrl_var(Vars, 'swrl:dbargs', DBArgs),
  rdf_equal(rdf:type, P),!,
  assert_rdf_class(Cls, Cls_rdf).
swrl_implication_pl(property(S,P,O), swrl_holds(S_var,P,O_var,DBArgs), Vars) :-
  swrl_var(Vars, S, S_var),
  swrl_var(Vars, O, O_var),
  swrl_var(Vars, 'swrl:dbargs', DBArgs).
  
%% swrl_condition_pl
swrl_condition_pl([A], A_pl, Vars) :-
  swrl_condition_pl(A, A_pl, Vars).
swrl_condition_pl([A,B|Xs], ','(A_pl,Ys), Vars) :-
  swrl_condition_pl(A, A_pl, Vars),
  swrl_condition_pl([B|Xs], Ys, Vars).
  
swrl_condition_pl(class(Cls,S), swrl:swrl_type_of(S_var,Cls_rdf,DBArgs), Vars) :-
  swrl_var(Vars, S, S_var),
  swrl_var(Vars, 'swrl:dbargs', DBArgs),
  assert_rdf_class(Cls, Cls_rdf).
swrl_condition_pl(property(S,P,O), swrl:swrl_triple(S_var,P,O_var,DBArgs), Vars) :-
  rdf_data_atom_(O,O_atom),
  swrl_var(Vars, S, S_var),
  swrl_var(Vars, O_atom, O_var),
  swrl_var(Vars, 'swrl:dbargs', DBArgs).
swrl_condition_pl(Builtin, Builtin_pl, Vars) :-
  swrl_builtin_pl(Builtin, Builtin_pl, Vars).

%% swrl_builtin_pl
swrl_builtin_pl(equal(S,O), (S_var = O_var), Vars) :-
  swrl_var(Vars, S, S_var),
  swrl_var(Vars, O, O_var).
swrl_builtin_pl(notEqual(S,O), (S_var \= O_var), Vars) :-
  swrl_var(Vars, S, S_var),
  swrl_var(Vars, O, O_var).
swrl_builtin_pl(lessThan(A,B), swrl_less(A_num,B_num), Vars) :-
  swrl_nums([A,B], [A_num,B_num], Vars).
swrl_builtin_pl(lessThanOrEqual(A,B), swrl_leq(A_num,B_num), Vars) :-
  swrl_nums([A,B], [A_num,B_num], Vars).
swrl_builtin_pl(greaterThan(A,B), swrl_greater(A_num,B_num), Vars) :-
  swrl_nums([A,B], [A_num,B_num], Vars).
swrl_builtin_pl(greaterThanOrEqual(A,B), swrl_geq(A_num,B_num), Vars) :-
  swrl_nums([A,B], [A_num,B_num], Vars).
%% math builtins
swrl_builtin_pl(add(A,B,C), swrl_plus(A_num,B_num,C_num), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).
swrl_builtin_pl(subtract(A,B,C), swrl_minus(A_num,B_num,C_num), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).
swrl_builtin_pl(multiply(A,B,C), swrl_multiply(A_num,B_num,C_num), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).
swrl_builtin_pl(divide(A,B,C), swrl_divide(A_num,B_num,C_num), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).
swrl_builtin_pl(mod(A,B,C), swrl_mod(A_num,B_num,C_num), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).
swrl_builtin_pl(pow(A,B,C), swrl_pow(A_num,B_num,C_num), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).
swrl_builtin_pl(abs(A,B), swrl_abs(B_num,A_num), Vars) :-
  swrl_nums([A,B], [A_num,B_num], Vars).
%% boolean builtins
swrl_builtin_pl(booleanNot(S), not(S_var), Vars) :-
  swrl_var(Vars, S, S_var).
%% string builtins
swrl_builtin_pl(stringConcat(A,B,C), swrl_atom_concat(C_atom,A_atom,B_atom), Vars) :-
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
swrl_builtin_pl(startsWith(A,X), swrl_atom_concat(X_atom, _, A_atom), Vars) :-
  swrl_atoms([A,X], [A_atom,X_atom], Vars).
swrl_builtin_pl(endsWith(A,X), swrl_atom_concat(_, X_atom, A_atom), Vars) :-
  swrl_atoms([A,X], [A_atom,X_atom], Vars).
% TODO (DB): support more builtins: matches, listConcat, member, length, ...

% FIXME: not sure why swrl_atoms above does not strip the literal
swrl_atom_concat(A0,A1,A2) :-
  (var(A0) ; rdf_data_atom_(A0,X0)),
  (var(A1) ; rdf_data_atom_(A1,X1)),
  (var(A2) ; rdf_data_atom_(A2,X2)),!,
  atom_concat(X0,X1,X2).

swrl_arithmetic(N,M,Goal) :- number_list(N,M), call(Goal).
swrl_leq(A,B)        :- swrl_arithmetic([A,B],[X,Y],(X =< Y)).
swrl_less(A,B)       :- swrl_arithmetic([A,B],[X,Y],(X < Y)).
swrl_geq(A,B)        :- swrl_arithmetic([A,B],[X,Y],(X >= Y)).
swrl_greater(A,B)    :- swrl_arithmetic([A,B],[X,Y],(X > Y)).
swrl_plus(A,B,C)     :- swrl_arithmetic([A,B,C],[X,Y,Z],(X is Y + Z)).
swrl_minus(A,B,C)    :- swrl_arithmetic([A,B,C],[X,Y,Z],(X is Y - Z)).
swrl_divide(A,B,C)   :- swrl_arithmetic([A,B,C],[X,Y,Z],(X is Y / Z)).
swrl_multiply(A,B,C) :- swrl_arithmetic([A,B,C],[X,Y,Z],(X is Y * Z)).
swrl_mod(A,B,C)      :- swrl_arithmetic([A,B,C],[X,Y,Z],(X is Y mod Z)).
swrl_pow(A,B,C)      :- swrl_arithmetic([A,B,C],[X,Y,Z],(X is Y ** Z)).
swrl_abs(A,B)        :- swrl_arithmetic([A,B],[X,Y],(X is abs(Y))).

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
  rdf_data_atom_(X, X_atom),
  catch(atom_number(X_atom,Y), _, fail),
  number_list(Xs,Ys).

%% assert class description in RDF triple store
%% TODO: move into some rdf module
assert_rdf_class(Descr, Descr) :- atom(Descr), !.
assert_rdf_class(Expr, Descr)  :-
  compound(Expr), rdf_node(Descr),
  assert_rdf_class_(Expr, Descr).
assert_rdf_class_(some(P,Cls), Descr) :-
  assert_rdf_class(Cls, ClsDescr),
  rdf_assert(Descr, rdf:type, owl:'Restriction'),
  rdf_assert(Descr, owl:someValuesFrom, ClsDescr),
  rdf_assert(Descr, owl:onProperty, P).
assert_rdf_class_(all(P,Cls), Descr) :-
  assert_rdf_class(Cls, ClsDescr),
  rdf_assert(Descr, rdf:type, owl:'Restriction'),
  rdf_assert(Descr, owl:allValuesFrom, ClsDescr),
  rdf_assert(Descr, owl:onProperty, P).
assert_rdf_class_(value(Value,P), Descr) :-
  rdf_assert(Descr, rdf:type, owl:'Restriction'),
  rdf_assert(Descr, owl:hasValue, Value),
  rdf_assert(Descr, owl:onProperty, P).
assert_rdf_class_(exactly(Num,P,Cls), Descr) :-
  assert_rdf_class(Cls, ClsDescr),
  rdf_assert(Descr, rdf:type, owl:'Restriction'),
  rdf_number_literal_(Num,Num_RDF),
  rdf_assert(Descr, owl:cardinality, Num_RDF),
  rdf_assert(Descr, owl:onProperty, P),
  rdf_assert(Descr, owl:onClass, ClsDescr).
assert_rdf_class_(min(Num,P,Cls), Descr) :-
  assert_rdf_class(Cls, ClsDescr),
  rdf_assert(Descr, rdf:type, owl:'Restriction'),
  rdf_number_literal_(Num,Num_RDF),
  rdf_assert(Descr, owl:minQualifiedCardinality, Num_RDF),
  rdf_assert(Descr, owl:onProperty, P),
  rdf_assert(Descr, owl:onClass, ClsDescr).
assert_rdf_class_(max(Num,P,Cls), Descr) :-
  assert_rdf_class(Cls, ClsDescr),
  rdf_assert(Descr, rdf:type, owl:'Restriction'),
  rdf_number_literal_(Num,Num_RDF),
  rdf_assert(Descr, owl:maxQualifiedCardinality, Num_RDF),
  rdf_assert(Descr, owl:onProperty, P),
  rdf_assert(Descr, owl:onClass, ClsDescr).
assert_rdf_class_(allOf(Cls), Descr) :-
  assert_rdf_list(Cls, ClsDescr),
  rdf_assert(Descr, rdf:type, owl:'Class'),
  rdf_assert(Descr, owl:intersectionOf, ClsDescr).
assert_rdf_class_(oneOf(Cls), Descr) :-
  assert_rdf_list(Cls, ClsDescr),
  rdf_assert(Descr, rdf:type, owl:'Class'),
  rdf_assert(Descr, owl:unionOf, ClsDescr).
assert_rdf_class_(not(Cls), Descr) :-
  assert_rdf_class(Cls, ClsDescr),
  rdf_assert(Descr, rdf:type, owl:'Class'),
  rdf_assert(Descr, owl:complementOf, ClsDescr).

rdf_number_literal_(Num,
  literal(type('http://www.w3.org/2001/XMLSchema#nonNegativeInteger',Atom))) :-
  ( atom(Num) -> Atom = Num ; term_to_atom(Num, Atom) ).

%% assert_rdf_list: assert list description in RDF triple store
%% TODO: move into some rdf module
assert_rdf_list([], Nil) :- rdf_equal(rdf:nil,Nil).
assert_rdf_list([X|Xs], Descr) :-
  rdf_node(Descr),
  assert_rdf_class(X, X_descr),
  assert_rdf_list(Xs, Xs_descr),
  rdf_assert(Descr, rdf:first, X_descr),
  rdf_assert(Descr, rdf:rest, Xs_descr).

%%
%% TODO: move into some rdf module
rdf_data_atom_(literal(type(_,Atom)),Atom) :- !.
rdf_data_atom_(literal(Atom),Atom) :- !.
rdf_data_atom_(Atom,Atom) :- !.


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
swrl_var_names(Var, Var) :-
  atom(Var),
  rdf_has(Var, rdf:type, swrl:'Variable').

%% Map between name of a SWRL variable and the corresponding
%% Prolog variable.
swrl_var(Vars, Name, PrologVar) :-
  atom(Name), member(var(Name,PrologVar), Vars), !.
swrl_var(Vars, var(X), PrologVar) :-
  swrl_var(Vars, X, PrologVar), !.
swrl_var(_, Name, Name).

set_vars(_, []).
set_vars(Vars, [var(Name,Var)|Rest]) :-
  swrl_var(Vars, Name, Var),
  set_vars(Vars, Rest).

swrl_vars_resolve([], [], _) :- !.
swrl_vars_resolve([var(X)|Xs], [Y|Ys], Vars) :- !,
  swrl_var(Vars, X, Y),
  swrl_vars_resolve(Xs, Ys, Vars).
swrl_vars_resolve([X|Xs], [X|Ys], Vars) :-
  swrl_vars_resolve(Xs, Ys, Vars).

swrl_atom(literal(type(_,A)), A, _) :- !.
swrl_atom(literal(A), A, _) :- !.
swrl_atom(var(A), A_val, Vars) :-
  swrl_var(Vars, A, A_var),
  swrl_atom(A_var,A_val,Vars).
swrl_atom(Atom, Atom, _) :-
  atom(Atom).
swrl_atoms([], [], _).
swrl_atoms([X|Xs], [Y|Ys], Vars) :-
  swrl_atom(X,Y,Vars),
  swrl_atoms(Xs,Ys,Vars).

swrl_atom_number(literal(type(_,A)), A_num, _) :-
  atom(A), !, catch(atom_number(A,A_num), _, fail).
swrl_atom_number(var(A), A_num, Vars) :-
  !, swrl_var(Vars, A, A_var),
  ( ground(A_var) ->
    swrl_atom_number(A_var, A_num, Vars)
  ; A_num = A_var ).
swrl_atom_number(A, A_num, _) :-
  atom(A), !, catch(atom_number(A,A_num), _, fail).
swrl_atom_number(A_num, A_num, _) :- number(A_num), !.

swrl_nums([],[],_).
swrl_nums([X|Xs],[Y|Ys],Vars) :-
  swrl_atom_number(X,Y,Vars),
  swrl_nums(Xs,Ys,Vars).

% TODO: convert units of datatype properties
%convert_to_unit(literal(type(InputType, InputVal)), OutputType, OutputVal).

%% swrl_satisfied(+Rule).
%% swrl_satisfied(+Rule,+Vars_user).
%
swrl_satisfied(Rule)            :- swrl_satisfied(Rule, [], _).
swrl_satisfied(Rule, Vars_user) :- swrl_satisfied(Rule, Vars_user, _).
swrl_satisfied([HeadAtom|Xs] :- Body, Vars_user, Vars) :-
  current_time(Now),
  % create list structure holding the variables
  swrl_vars([HeadAtom|Xs] :- Body, Vars_rule), Vars = [var('swrl:dbargs',_{during:Now})|Vars_rule],
  set_vars(Vars, Vars_user),
  % create Prolog structure of condition term
  swrl_rule_pl(HeadAtom :- Body,  _ :- Cond_pl, Vars),
  call( Cond_pl ).

		 /************************************
		  *   Projection of SWRL rules       *
		  ************************************/

%% swrl_project(+Rule).
%% swrl_project(+Rule,+Vars_user).
%
% Project implications of a SWRL rule into the RDF triple store.
% The rule is described by the individual `Descr` in the RDF triple store
% or by the SWRL term `Rule`.
%
swrl_project(Rule) :- swrl_project(Rule, []).
swrl_project([HeadAtom|Xs] :- Body, Vars_user) :-
  findall( Binding,
    swrl_satisfied([HeadAtom|Xs] :- Body, Vars_user, Binding),
    Bindings ),
  ( Bindings = [] ; (
    member(Vars, Bindings),
    swrl_project_([HeadAtom|Xs], Vars) )).

swrl_project_([], _).
swrl_project_([Atom|Xs], Vars) :-
  swrl_atom_project(Atom,Vars),
  swrl_project_(Xs, Vars).

%%
swrl_atom_project(class(Cls,S), Vars) :-
  swrl_var(Vars, S, S_var),
  swrl_class_atom_project(S_var, Cls).

swrl_atom_project(property(S,P,O), Vars) :-
  rdf_data_atom_(O, O_atom),
  swrl_var(Vars, S, S_var),
  swrl_var(Vars, O_atom, O_var),
  swrl_assert_triple(S_var, P, O_var).

%%
swrl_class_atom_project(Subject, Cls_rdf) :-
  assert_rdf_class(Cls_rdf, Cls),
  swrl_assert_triple(Subject, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', Cls).
