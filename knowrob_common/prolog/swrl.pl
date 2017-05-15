/** <module> SWRL rules

  Copyright (C) 2017 Daniel Beßler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@author Daniel Beßler
@license BSD
*/
:- module(swrl,
    [
      rdf_swrl_rule/2,
      rdf_swrl_atom/2,
      rdf_swrl_rule_type/2,
      rdf_swrl_target_concept/2,
      rdf_swrl_target_variable/2,
      rdf_swrl_load/0,
      rdf_swrl_load/1,
      rdf_swrl_projection/1,
      swrl_assert/1,
      swrl_phrase/2,
      swrl_phrase_assert/1,
      swrl_individual_of/2,
      swrl_individual_of/3,
      swrl_holds/3,
      swrl_holds/4,
      swrl_projection/1
    ]).

:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(rdfs, 'http://www.w3.org/2000/01/rdf-schema#', [keep(true)]).
:- rdf_db:rdf_register_ns(xml, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_owl')).
:- use_module(library('knowrob_temporal')).

:- rdf_meta swrl_individual_of(r,r),
            swrl_individual_of(r,r,r),
            swrl_holds(r,r,r,r),
            swrl_holds(r,r,r),
            swrl_projection(r),
            swrl_match_instance(r,r,r).
:- dynamic  swrl_holds/4,
            call_mutex/2.

% expand holds predicate
% FIXME(DB): will not fully work when interval unspecified
%       - find interval intersection during unification?
knowrob_temporal:holds(S,P,O,I) :- swrl_holds(S,P,O,I).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% RDF-based SWRL representation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% rdf_swrl_rule(?Descr, ?Term)
%
% @Descr The RDF description of a SWRL rule
% @Term Prolog term describing the SWRL rule
%
rdf_swrl_rule(Descr, Head :- Body) :-
  rdf_has(Descr, rdf:type, swrl:'Imp'),
  rdf_has(Descr, swrl:body, BodyDescr),
  rdf_has(Descr, swrl:head, HeadDescr),
  rdf_swrl_atom(BodyDescr, Body),
  rdf_swrl_atom(HeadDescr, Head).

%% rdf_swrl_atom(?Descr, ?Term)
%
% Used for mapping RDF descriptions of SWRL rules to
% Prolog-encoded representation.
% Can either be used for querying the RDF description that
% contains particular classes, restrictions, and properties,
% or to generate a Prolog list representation of the RDF description.
%
% @Descr The RDF description of a SWRL rule
% @Term Prolog term describing a SWRL atom
%
rdf_swrl_atom(Descr, List) :-
  rdf_has(Descr, rdf:type, swrl:'AtomList'),
  rdf_swrl_atom_list(Descr, List).

rdf_swrl_atom(Descr, List) :-
  rdf_has(Descr, rdf:type, rdf:'List'),
  rdf_swrl_atom_list(Descr, List).

rdf_swrl_atom(Cls, Cls) :-
  rdf_has(Cls, rdf:type, owl:'Class').

rdf_swrl_atom(literal(type(
  'http://www.w3.org/2001/XMLSchema#anyURI', Val)), Val) :- !.
rdf_swrl_atom(literal(type(Type, Val)),
              literal(type(Type, Val))).
rdf_swrl_atom(literal(Val), literal(Val)).

rdf_swrl_atom(Descr, var(Descr)) :-
  rdf_has(Descr, rdf:type, swrl:'Variable').

rdf_swrl_atom(Descr, class(ClsTerm,S)) :-
  rdf_has(Descr, rdf:type, swrl:'ClassAtom'),
  rdf_has(Descr, swrl:classPredicate, Cls),
  rdf_has(Descr, swrl:argument1, S),
  rdf_swrl_class_atom(Cls, ClsTerm).

rdf_swrl_atom(Descr, property(S,P,O)) :-
  rdf_has(Descr, rdf:type, swrl:'IndividualPropertyAtom'),
  rdf_has(Descr, swrl:propertyPredicate, P),
  rdf_has(Descr, swrl:argument1, S),
  rdf_has(Descr, swrl:argument2, O).

rdf_swrl_atom(Descr, property(S,P,O)) :-
  rdf_has(Descr, rdf:type, swrl:'DatavaluedPropertyAtom'),
  rdf_has(Descr, swrl:propertyPredicate, P),
  rdf_has(Descr, swrl:argument1, S),
  rdf_has(Descr, swrl:argument2, O).

rdf_swrl_atom(Descr, Atom) :-
  rdf_has(Descr, rdf:type, swrl:'BuiltinAtom'),
  rdf_has(Descr, swrl:builtin, BuiltinIri),
  rdf_split_url(_, Builtin, BuiltinIri),
  rdf_has(Descr, swrl:arguments, Args),
  rdf_swrl_atom(Args, ArgsAtom),
  (  is_list(ArgsAtom)
  -> Atom =.. [Builtin|ArgsAtom]
  ;  Atom =.. [Builtin,ArgsAtom] ).

rdf_swrl_class_atom(Cls, not(ClsTerm)) :-
  rdf_has(Cls, owl:complementOf, ClsDescr), !,
  rdf_swrl_class_atom(ClsDescr, ClsTerm).
rdf_swrl_class_atom(Cls, Cls).

rdf_swrl_atom_list(Descr, []) :-
  rdf_equal(Descr, rdf:'nil').
rdf_swrl_atom_list(Descr, [First|Rest]) :-
  rdf_has(Descr, rdf:first, FirstDescr),
  rdf_swrl_atom(FirstDescr, First),
  rdf_has(Descr, rdf:rest, RestDescr),
  rdf_swrl_atom_list(RestDescr, Rest).

%% rdf_swrl_rule_type(?Descr, ?RuleType)
%
% @Descr The RDF description of a SWRL rule
% @RuleType The IRI of a SWRL rule type
%
rdf_swrl_rule_type(Descr, RuleType) :-
  rdf_has(Descr, knowrob:swrlType, literal(type(_,RuleType))).

%% rdf_swrl_target_concept(?Descr, ?RuleType)
%
% @Descr The RDF description of a SWRL rule
% @RuleType The IRI of a SWRL rule target concept (i.e., the concept to which the rule is applied)
%
rdf_swrl_target_concept(Descr, TargetConcept) :-
  rdf_has(Descr, knowrob:swrlTargetConcept, literal(type(_,TargetConcept))).

%% rdf_swrl_target_variable(?Descr, ?RuleVar)
%
% @Descr The RDF description of a SWRL rule
% @RuleType The IRI of a SWRL rule target concept (i.e., the concept to which the rule is applied)
%
rdf_swrl_target_variable(Descr, RuleVar) :-
  rdf_has(Descr, knowrob:swrlTargetVariable, literal(type(_,RuleVar))).

%% rdf_swrl_load
% 
% Reads SWRL rules from RDF triple store and
% converts them to native Prolog rules.
%
rdf_swrl_load :-
  forall( rdf_swrl_rule(_, Rule), swrl_assert(Rule) ).

rdf_swrl_load(Descr) :-
  atom(Descr),
  rdf_has(Descr, rdf:type, swrl:'Imp'), !,
  rdf_swrl_rule(Descr, Rule),
  swrl_assert(Rule).

rdf_swrl_load(Id) :-
  once(( rdf_has(Descr, rdfs:label, literal(type(_,Id))) ;
         rdf_has(Descr, rdfs:label, literal(Id)) )),
  rdf_has(Descr, rdf:type, swrl:'Imp'),
  rdf_swrl_load(Descr).

%% rdf_swrl_projection
% 
rdf_swrl_projection(Descr) :-
  atom(Descr),
  rdf_has(Descr, rdf:type, swrl:'Imp'), !,
  rdf_swrl_rule(Descr,Rule),
  swrl_projection(Rule).

rdf_swrl_projection(Id) :-
  once(( rdf_has(Descr, rdfs:label, literal(type(_,Id))) ;
         rdf_has(Descr, rdfs:label, literal(Id)) )),
  rdf_has(Descr, rdf:type, swrl:'Imp'),
  rdf_swrl_projection(Descr).


%% rdf_class_pl
% 
% Read Prolog representation of class from RDF triple store
% 
rdf_class_pl(Cls, not(ClsTerm)) :-
  rdf_has(Cls, owl:complementOf, ClsDescr), !,
  rdf_class_pl(ClsDescr, ClsTerm).
rdf_class_pl(Cls, allOf(Classes)) :-
  rdf_has(Cls, owl:intersectionOf, ClsDescr), !,
  rdf_class_list_pl(ClsDescr, Classes).
rdf_class_pl(Cls, oneOf(Classes)) :-
  rdf_has(Cls, owl:unionOf, ClsDescr), !,
  rdf_class_list_pl(ClsDescr, Classes).
rdf_class_pl(Cls, Restr) :-
  rdf_has(Cls, rdf:type, owl:'Restriction'), !,
  rdf_restriction_pl(Cls, Restr).
rdf_class_pl(Cls, Cls) :- atom(Cls), !.

%% rdf_restriction_pl
% 
% Read Prolog representation of restriction from RDF triple store
% 
rdf_restriction_pl(Descr, some(P,Cls_pl)) :-
  rdf_has(Descr, owl:someValuesFrom, Cls), !,
  rdf_has(Descr, owl:onProperty, P),
  rdf_class_pl(Cls, Cls_pl).
rdf_restriction_pl(Descr, all(P,Cls_pl)) :-
  rdf_has(Descr, owl:allValuesFrom, Cls), !,
  rdf_has(Descr, owl:onProperty, P),
  rdf_class_pl(Cls, Cls_pl).
rdf_restriction_pl(Descr, value(Value,P)) :- 
  ( rdf_has(Descr, owl:hasValue, literal(type(_,Value))) ;
    rdf_has(Descr, owl:hasValue, literal(Value)) ;
    rdf_has(Descr, owl:hasValue, Value) ),
  rdf_has(Descr, owl:onProperty, P).
rdf_restriction_pl(Descr, exactly(Num,P,Cls_pl)) :- 
  ( rdf_has(Descr, owl:cardinality, literal(type(_,Num))) ;
    rdf_has(Descr, owl:qualifiedCardinality, literal(type(_,Num))) ), !,
  rdf_has(Descr, owl:onProperty, P),
  ( rdf_has(Descr, owl:onClass, Cls) ; Cls='http://www.w3.org/2002/07/owl#Thing' ),
  rdf_class_pl(Cls, Cls_pl).
rdf_restriction_pl(Descr, max(Num,P,Cls_pl)) :- 
  rdf_has(Descr, owl:maxQualifiedCardinality, literal(type(_,Num))), !,
  rdf_has(Descr, owl:onProperty, P),
  ( rdf_has(Descr, owl:onClass, Cls) ; Cls='http://www.w3.org/2002/07/owl#Thing' ),
  rdf_class_pl(Cls, Cls_pl).
rdf_restriction_pl(Descr, min(Num,P,Cls_pl)) :- 
  rdf_has(Descr, owl:minQualifiedCardinality, literal(type(_,Num))), !,
  rdf_has(Descr, owl:onProperty, P),
  ( rdf_has(Descr, owl:onClass, Cls) ; Cls='http://www.w3.org/2002/07/owl#Thing' ),
  rdf_class_pl(Cls, Cls_pl).

rdf_class_list_pl(Descr, []) :-
  rdf_equal(Descr, rdf:'nil').
rdf_class_list_pl(Descr, [First|Rest]) :-
  rdf_has(Descr, rdf:first, FirstDescr),
  rdf_class_pl(FirstDescr, First),
  rdf_has(Descr, rdf:rest, RestDescr),
rdf_class_list_pl(RestDescr, Rest).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Prolog-encoded SWRL representation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% swrl_individual_of
%
swrl_individual_of(S,Cls) :-
  current_time(Now),
  swrl_individual_of(S,Cls,Now).
swrl_individual_of(S,Cls,I) :-
  ground(S), ground(Cls), !,
  once(owl_individual_of_during(S,Cls,I)).
swrl_individual_of(S,Cls,I) :-
  owl_individual_of_during(S,Cls,I).

%% swrl_holds
%
swrl_holds(S,P,O) :-
  current_time(Now),
  swrl_holds(S,P,O,Now).

%% swrl_assert
%
swrl_assert([] :- _).
swrl_assert([HeadAtom|Xs] :- Body) :-
  swrl_vars([HeadAtom] :- Body, Vars),
  swrl_rule_pl(HeadAtom :- Body, Rule_pl, [var('swrl:interval',_)|Vars]),
  assertz( Rule_pl ),
  swrl_assert(Xs :- Body).

%% swrl_rule_pl
%
swrl_rule_pl(Fact :- [], Fact_pl, Vars) :-
  !, swrl_implication_pl(Fact, Fact_pl, Vars).
swrl_rule_pl(Impl :- Cond, Impl_pl :- Cond_pl, Vars) :-
  swrl_implication_pl(Impl, Impl_pl, Vars),
  swrl_condition_pl(Cond, Cond_pl_, Vars),
  % avoid that rules call themself in conditions
  random(0, 999999999, MutexId),
  Cond_pl = (
    thread_self(ThreadId),
    with_call_mutex(MutexId, ThreadId, Cond_pl_)
  ).

with_call_mutex(MutexId, ThreadId, Goal) :-
  \+ call_mutex(MutexId,ThreadId),
  setup_call_cleanup(
    assert(call_mutex(MutexId,ThreadId)),
    Goal,
    retract(call_mutex(MutexId,ThreadId))
  ).

%% swrl_implication_pl
%
swrl_implication_pl(class(Cls,S), swrl_holds(S_var,P,Cls_rdf,I_var), Vars) :-
  swrl_var(Vars, S, S_var),
  swrl_var(Vars, 'swrl:interval', I_var),
  rdf_equal(rdf:type, P),
  assert_rdf_class(Cls, Cls_rdf).
swrl_implication_pl(property(S,P,O), swrl_holds(S_var,P,O_var,I_var), Vars) :-
  swrl_var(Vars, S, S_var),
  swrl_var(Vars, O, O_var),
  swrl_var(Vars, 'swrl:interval', I_var).

%% swrl_condition_pl
%
swrl_condition_pl([A], A_pl, Vars) :-
  swrl_condition_pl(A, A_pl, Vars).
swrl_condition_pl([A,B|Xs], ','(A_pl,Ys), Vars) :-
  swrl_condition_pl(A, A_pl, Vars),
  swrl_condition_pl([B|Xs], Ys, Vars).
% FIXME(DB): this won't work when subject unbound. Better use OWL class and let `owl_individual_of` handle it.
swrl_condition_pl(class(not(Cls),S), (\+ ClsTerm), Vars) :-
  !, swrl_condition_pl(class(Cls,S), ClsTerm, Vars).
swrl_condition_pl(class(Cls,S), swrl_individual_of(S_var,Cls_rdf,I_var), Vars) :-
  swrl_var(Vars, S, S_var),
  swrl_var(Vars, 'swrl:interval', I_var),
  assert_rdf_class(Cls, Cls_rdf).
swrl_condition_pl(property(S,P,O), holds(S_var,P,O_var,I_var), Vars) :-
  swrl_var(Vars, S, S_var),
  swrl_var(Vars, O, O_var),
  swrl_var(Vars, 'swrl:interval', I_var).
swrl_condition_pl(Builtin, Builtin_pl, Vars) :-
  swrl_builtin_pl(Builtin, Builtin_pl, Vars).

%%%%%%%%%%%%
%%%%%%%%%%%%
%%%% SWRL builtins
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
%% SWRL math builtins
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
%% SWRL boolean builtins
swrl_builtin_pl(booleanNot(S), not(S_var), Vars) :-
  swrl_var(Vars, S, S_var).
%% SWRL string builtins
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
% TODO: support for more builtins:
%    matches, listConcat, member, length, first, rest, empty

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
  strip_literal_type(X, X_striped),
  catch(atom_number(X_striped,Y), _, fail),
  number_list(Xs,Ys).

%% assert_rdf_class
%
% Bind IRI of RDF class description to `Descr`
%
assert_rdf_class(Descr, Descr) :- atom(Descr), !.
assert_rdf_class(Expr, Descr)  :-
  compound(Expr), rdf_node(Descr),
  assert_rdf_class_(Expr, Descr).

assert_rdf_class_(some(P,Cls), Descr) :-
  assert_rdf_class(Cls, ClsDescr),
  rdf_assert(Descr, owl:someValuesFrom, ClsDescr),
  rdf_assert(Descr, owl:onProperty, P).
assert_rdf_class_(all(P,Cls), Descr) :-
  assert_rdf_class(Cls, ClsDescr),
  rdf_assert(Descr, owl:allValuesFrom, ClsDescr),
  rdf_assert(Descr, owl:onProperty, P).
assert_rdf_class_(value(Value,P), Descr) :-
  rdf_assert(Descr, owl:hasValue, Value),
  rdf_assert(Descr, owl:onProperty, P).
assert_rdf_class_(exactly(Num,P,Cls), Descr) :-
  assert_rdf_class(Cls, ClsDescr),
  rdf_assert(Descr, owl:cardinality, Num),
  rdf_assert(Descr, owl:onProperty, P),
  rdf_assert(Descr, owl:onClass, ClsDescr).
assert_rdf_class_(min(Num,P,Cls), Descr) :-
  assert_rdf_class(Cls, ClsDescr),
  rdf_assert(Descr, owl:minQualifiedCardinality, Num),
  rdf_assert(Descr, owl:onProperty, P),
  rdf_assert(Descr, owl:onClass, ClsDescr).
assert_rdf_class_(max(Num,P,Cls), Descr) :-
  assert_rdf_class(Cls, ClsDescr),
  rdf_assert(Descr, owl:maxQualifiedCardinality, Num),
  rdf_assert(Descr, owl:onProperty, P),
  rdf_assert(Descr, owl:onClass, ClsDescr).
assert_rdf_class_(allOf(Cls), Descr) :-
  assert_rdf_list(Cls, ClsDescr),
  rdf_assert(Descr, owl:intersectionOf, ClsDescr).
assert_rdf_class_(oneOf(Cls), Descr) :-
  assert_rdf_list(Cls, ClsDescr),
  rdf_assert(Descr, owl:unionOf, ClsDescr).
assert_rdf_class_(not(Cls), Descr) :-
  assert_rdf_class(Cls, ClsDescr),
  rdf_assert(Descr, owl:complementOf, ClsDescr).

assert_rdf_list([], rdf:nil).
assert_rdf_list([X|Xs], Descr) :-
  rdf_node(Descr),
  assert_rdf_class(X, X_descr),
  assert_rdf_list(Xs, Xs_descr),
  rdf_assert(Descr, rdf:first, X_descr),
  rdf_assert(Descr, rdf:rest, Xs_descr).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Variable mapping
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% swrl_vars(+Rule, ?Variables)
%
% @Head List of Prolog-encoded SWRL atoms
% @Body List of Prolog-encoded SWRL atoms
% @Variables Mapping between SWRL variable names and Prolog variables
%
swrl_vars(Head :- Body, Variables) :-
  findall(VarName, (
    (member(Atom,Head) ; member(Atom,Body)),
    swrl_var_names(Atom,VarName)
  ), VarNames),
  list_to_set(VarNames, VarNamesUnique),
  maplist(swrl_var_pl(_), VarNamesUnique,Variables).
swrl_var_pl(_,Y,var(Y,_)).

swrl_var_names(var(Var), Var) :- atom(Var), !.
swrl_var_names(Term, Var) :-
  compound(Term), !,
  Term =.. [_|Args],
  member(Arg, Args),
  swrl_var_names(Arg, Var).
swrl_var_names(Var, Var) :-
  atom(Var),
  rdf_has(Var, rdf:type, swrl:'Variable').

%% swrl_var(+Vars, +Name, -PrologVar)
%
% @Vars Mapping between SWRL variable names and Prolog variables
% @Name A SWRL variable name
% @PrologVar The mapped Prolog variable
%
swrl_var(Vars, Name, PrologVar) :-
  atom(Name), nth0(_, Vars, var(Name,PrologVar)), !.
swrl_var(Vars, var(X), PrologVar) :-
  swrl_var(Vars, X, PrologVar), !.
swrl_var(_, Name, Name).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

swrl_atom(literal(type(_,A)), A, _) :- !.
swrl_atom(literal(A), A, _).
swrl_atom(var(A), A_val, Vars) :-
  swrl_var(Vars, A, A_var),
  swrl_atom(A_var,A_val,Vars).
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Atom projection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% swrl_projection(+Rule)
%
% Project implications of a SWRL rule into the RDF triple store managed by KnowRob.
% The rule is described by the individual `Descr` in the RDF triple store
% or by the SWRL term `Rule`
% Triples are not asserted to individuals directly but to their temporal parts.
% Negative atoms such as `(not Driver)(?x)` are projected to the temporal parts
% by specifying the end time of the corresponding time interval.
% Thus, no facts are retracted in the projection.
%
swrl_projection([HeadAtom|Xs] :- Body) :-
  current_time(Now),
  swrl_vars([HeadAtom|Xs] :- Body, Vars), Vars_ = [var('swrl:interval',Now)|Vars],
  swrl_rule_pl(HeadAtom :- Body,  _ :- Cond_pl, Vars_),
  call( Cond_pl ),
  swrl_projection([HeadAtom|Xs], Vars_).

swrl_projection([], _).
swrl_projection([HeadAtom|Xs], Vars) :-
  % check condition, bind vars
  (  swrl_atom_holds(HeadAtom,Vars)
  -> true
  ;  swrl_atom_project(HeadAtom,Vars) ),
  swrl_projection(Xs, Vars).

% Check if implication already fulfilled in the RDF triple store
% (SWRL rules are ignored here!)
swrl_atom_holds(class(Cls,S), Vars) :-
  swrl_var(Vars, S, S_var),
  swrl_class_atom_satisfied(S_var,Cls).
swrl_atom_holds(property(S,P,O), Vars) :-
  swrl_var(Vars, S, S_var),
  swrl_var(Vars, O, O_var),
  owl_has(S_var,P,O_var).

%% Project implication atoms
swrl_atom_project(class(Cls,S), Vars) :- !,
  swrl_var(Vars, S, S_var),
  swrl_class_atom_project(S_var, Cls).
swrl_atom_project(property(S,P,O), Vars) :-
  strip_literal_type(O, O_val),
  swrl_var(Vars, S, S_var),
  swrl_var(Vars, O_val, O_var),
  assert_temporal_part(S_var, P, O_var).

%%%%%%%%%%%%%%%%%%%%%%%
%% Class atoms

swrl_class_atom_project(S, not(allOf(Xs)))        :- once(( member(X,Xs), swrl_class_atom_project(S,not(X)) )).
swrl_class_atom_project(S, not(oneOf(Xs)))        :- forall( member(X,Xs), swrl_class_atom_project(S,not(X)) ).
swrl_class_atom_project(S, not(value(P,O)))       :- forall( assert_temporal_part_end(S,P,O), true ).
swrl_class_atom_project(S, not(exactly(N,P,Cls))) :- swrl_class_atom_project(S, exactly(M,P,Cls)), M is N+1.
swrl_class_atom_project(S, not(min(N,P,Cls)))     :- swrl_class_atom_project(S, max(M,P,Cls)), M is N-1.
swrl_class_atom_project(S, not(max(N,P,Cls)))     :- swrl_class_atom_project(S, min(M,P,Cls)), M is N+1.
swrl_class_atom_project(S, not(some(P,Cls)))      :- swrl_class_atom_project(S, exactly(0,P,Cls)).
swrl_class_atom_project(S, not(all(P,Cls)))       :-
  once((
      swrl_relation_values(S,P,Cls,Os),
      member(O,Os),
      swrl_class_atom_satisfied(O,Cls)
    ) ; (
      % assert a random individual
      rdf_phas(P,rdfs:range,Range),
      owl_individual_of(O, Range),
      \+ swrl_class_atom_satisfied(S,Cls),
      assert_temporal_part(S,P,O)
  )).
swrl_class_atom_project(S, not(Cls)) :-
  atom(Cls), rdfs_individual_of(Cls, owl:'Class'),
  assert_temporal_part_end(S, rdf:'type', nontemporal(Cls)).

swrl_class_atom_project(S, allOf(Classes)) :-
  % Project each class description in the intersection
  forall( member(Cls, Classes), swrl_class_atom_project(S, Cls) ).
swrl_class_atom_project(S, oneOf(Classes)) :-
  % Project one random class description in the union
  once(( member(Cls, Classes), swrl_class_atom_project(S, Cls) )).

swrl_class_atom_project(S, all(P,Cls)) :-
  % Retract all values that are not an instance of Cls
  findall(O, (
      owl_has(S,P,O),
      \+ swrl_class_atom_satisfied(O,Cls)
  ), Values),
  forall( member(O,Values),
          assert_temporal_part_end(S,P,O) ).
swrl_class_atom_project(S, some(P,Cls)) :-
  % Assert random object fulfilling the relation
  once(( swrl_class_atom_satisfied(S, some(P,Cls)) ;
         swrl_assert_random(S, P, Cls, 1) )).

swrl_class_atom_project(S, exactly(0,P,Cls)) :- !,
  % Retract all facts about objects fulfilling the class description
  swrl_relation_values(S,P,Cls,Values),
  forall( member(O,Values), assert_temporal_part_end(S,P,O) ).
swrl_class_atom_project(S, exactly(N,P,Cls)) :-
  % End or begin some random temporal parts with temporal relation P so that the number of
  % active temporal parts is equal to qualified cardinality.
  swrl_cardinality_diff(S, P, Cls, N, Diff),
  ( Diff >= 0 ->
    swrl_retract_random(S, P, Cls, Diff) ;
    swrl_assert_random(S, P, Cls, Diff) ).

swrl_class_atom_project(S, max(N,P,Cls)) :-
  % End some random temporal parts with temporal relation P so that the number of
  % active temporal parts is below specified max cardinality.
  swrl_cardinality_diff(S, P, Cls, N, Diff),
  swrl_retract_random(S, P, Cls, Diff).
swrl_class_atom_project(S, min(N,P,Cls)) :-
  swrl_cardinality_diff(S, P, Cls, N, Diff),
  swrl_assert_random(S, P, Cls, Diff).

swrl_class_atom_project(S, Cls) :-
  atom(Cls),
  % unwrap class descriptions for projection
  rdf_class_pl(Cls, Cls_pl),
  ( atom(Cls_pl) -> (
    rdfs_individual_of(Cls_pl, owl:'Class'),
    assert_temporal_part(S, rdf:'type', nontemporal(Cls_pl))
  ) ; (
    swrl_class_atom_project(S, Cls_pl)
  )).

%% retract random objects that fulfills relation
swrl_retract_random(_, _, _, N) :- N =< 0, !.
swrl_retract_random(S, P, Cls, N) :-
  once((
    current_temporal_part(S,TemporalPart),
    rdf_has(TemporalPart, knowrob:'temporalProperty', P),
    temporal_part_value(TemporalPart, P, O),
    atom(O),
    swrl_class_atom_satisfied(O,Cls),
    assert_temporal_part_end(S,P,O)
  )),
  M is N - 1, swrl_retract_random(S, P, Cls, M).

%% assert random objects that fulfills relation
swrl_assert_random(_, _, _, N) :- N >= 0, !.
swrl_assert_random(S, P, Cls, N) :-
  swrl_class_atom_satisfied(O,Cls),
  \+ ( % only assert if temporal part with this relation does not exist
    current_temporal_part(S,TemporalPart),
    rdf_has(TemporalPart, knowrob:'temporalProperty', P),
    temporal_part_value(TemporalPart, P, O)
  ),
  assert_temporal_part(S,P,O),
  M is N + 1, swrl_assert_random(S, P, Cls, M).

%% compute set of objects fulfilling a relation
swrl_relation_values(S,P,Cls,Values) :-
  findall(O, (
      owl_has(S,P,O),
      swrl_class_atom_satisfied(O,Cls)
  ), Os),
  list_to_set(Os,Values).

%% compute actual cardinality of a relation
swrl_relation_count(S,P,Cls,Count) :-
  swrl_relation_values(S,P,Cls,Values),
  length(Values, Count).

swrl_cardinality_diff(S, P, Cls, Num, Diff) :-
  swrl_relation_count(S, P, Cls, CurrentNum),
  Diff is CurrentNum - Num.

%% Class atoms
swrl_class_atom_satisfied(S, not(Cls)) :-
  \+ swrl_class_atom_satisfied(S,Cls).
swrl_class_atom_satisfied(S, allOf(Cls)) :-
  forall( member(X,Cls), swrl_class_atom_satisfied(S, X) ).
swrl_class_atom_satisfied(S, oneOf(Cls)) :-
  once(( member(X,Cls), swrl_class_atom_satisfied(S, X) )).
swrl_class_atom_satisfied(S, some(P,Cls)) :-
  once(( owl_has(S,P,O), swrl_class_atom_satisfied(O,Cls) )).
swrl_class_atom_satisfied(S, all(P,Cls)) :-
  forall( owl_has(S,P,O), swrl_class_atom_satisfied(O,Cls) ).
swrl_class_atom_satisfied(S, min(N,P,Cls)) :-
  findall(O, (owl_has(S,P,O), swrl_class_atom_satisfied(O,Cls)), Os ),
  length(Os, Os_length), atom_number(N,N_val), Os_length >= N_val.
swrl_class_atom_satisfied(S, max(N,P,Cls)) :-
  findall(O, (owl_has(S,P,O), swrl_class_atom_satisfied(O,Cls)), Os ),
  length(Os, Os_length), atom_number(N,N_val), Os_length =< N_val.
swrl_class_atom_satisfied(S, exactly(N,P,Cls)) :-
  findall(O, (owl_has(S,P,O), swrl_class_atom_satisfied(O,Cls)), Os ),
  atom_number(N,N_val), length(Os,N_val).
swrl_class_atom_satisfied(S, Cls) :-
  atom(Cls), owl_individual_of(S, Cls).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% SWRL phrase to Prolog term and vice versa
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% swrl_phrase(?Term, ?Expr)
%
% Term is unified with the parse tree of parsing Expr as SWRL rule.
% Can be used to generate human readable syntax for Prolog SWRL terms,
% but also for generating Prolog term representation of human readable
% SWRL expression.
%
% For example, `swrl_phrase(Term, 'Driver(?p) -> Person(?p)')`
% parses the expression and generates the term:
% 
%   Term = [ class(test_swrl:'Person',var(p)) ] :-
%                [ class(test_swrl:'Driver',var(p)) ]
%
% @Term A Prolog term describing a SWRL rule
% @Expr A SWRL describtion in human readable syntax
%
swrl_phrase(Term, Expr) :-
  ground(Term),
  phrase(swrl_parser(Term), Tokens),
  swrl_tokens_format(Tokens, Expr).
swrl_phrase(Term, Expr) :-
  atom(Expr),
  tokenize_atom(Expr, Tokens),
  phrase(swrl_parser(Term), Tokens).

%% swrl_phrase_assert(?Phrase)
%
swrl_phrase_assert(Phrase) :-
  swrl_phrase(Term, Phrase),
  swrl_assert(Term).

swrl_tokens_format(Tokens, Expr) :-
  once(swrl_tokens_format_(Tokens, Tokens2)),
  atomic_list_concat(Tokens2,'',Expr).

swrl_tokens_format_([],[]).
swrl_tokens_format_(ToksIn,ToksOut) :-
  swrl_token_spaces(ToksIn, ToksOut1, Rest),
  swrl_tokens_format_(Rest,ToksOut2),
  append(ToksOut1, ToksOut2, ToksOut).

swrl_token_spaces(['-','>'|Xs], [' -> '], Xs).
swrl_token_spaces(['?'|Xs], ['?'], Xs).
swrl_token_spaces(['or'|Xs], ['or '], Xs).
swrl_token_spaces(['and'|Xs], ['and '], Xs).
swrl_token_spaces(['('|Xs], ['('], Xs).
swrl_token_spaces([Tok,','|Xs], [Tok,', '],Xs).
swrl_token_spaces([','|Xs], [', '],Xs).
swrl_token_spaces([Tok,')'|Xs], [Tok,')'], Xs).
swrl_token_spaces([')'|Xs], [')'], Xs).
swrl_token_spaces(['"',Tok,'"'|Xs], ['"',Tok,'"'], Xs).
swrl_token_spaces([not,'('|Xs], [not,' ('], Xs).
swrl_token_spaces([Tok,'('|Xs], [Tok,'('], Xs).
swrl_token_spaces([X|Xs], [X,' '], Xs).
swrl_token_spaces([], [], []).

swrl_parser([])   --> [].
swrl_parser(Tree) --> swrl_rule(Tree).

swrl_rule(Head :- Body)       --> swrl_conjunction(Body), ['-'], ['>'], swrl_conjunction(Head). 
swrl_conjunction([Atom|Rest]) --> swrl_literal(Atom), [','], swrl_conjunction(Rest).
swrl_conjunction([Atom])      --> swrl_literal(Atom).

swrl_literal(class(A,B)) -->
  swrl_class_atom(A), ['('], swrl_subject(B), [')'].
swrl_literal(property(S,P,O)) -->
  swrl_property(P), ['('], swrl_subject(S), [','], swrl_data_object(O), [')'],
  { rdfs_individual_of(P, owl:'DatatypeProperty') }.
swrl_literal(property(S,P,O)) -->
  swrl_property(P), ['('], swrl_subject(S), [','], swrl_object(O), [')'],
  { rdfs_individual_of(P, owl:'ObjectProperty') }.
swrl_literal(BuiltinTerm) -->
  { (ground(BuiltinTerm), BuiltinTerm =.. [Predicate|Args]) ; true },
  swrl_builtin(Predicate), ['('], swrl_builtin_args(Args), [')'],
  { BuiltinTerm =.. [Predicate|Args] }.

swrl_class_atom(not(Cls))           --> ['('], ['not'], swrl_class_atom(Cls), [')'].
swrl_class_atom(allOf([X|Xs]))      --> ['('], swrl_class_atom_terminal(X), ['and'], swrl_class_intersection(Xs), [')'].
swrl_class_atom(oneOf([X|Xs]))      --> ['('], swrl_class_atom_terminal(X), ['or'], swrl_class_union(Xs), [')'].
swrl_class_atom(some(P,Cls))        --> ['('], swrl_property(P), ['some'], swrl_class_atom(Cls), [')'].
swrl_class_atom(all(P,Cls))         --> ['('], swrl_property(P), ['all'], swrl_class_atom(Cls), [')'].
swrl_class_atom(value(P,Value))     --> ['('], swrl_property(P), ['value'], swrl_value(Value), [')'].
swrl_class_atom(exactly(Num,P,Cls)) --> ['('], swrl_property(P), ['exactly'], swrl_number(Num), swrl_class_atom_terminal(Cls), [')'].
swrl_class_atom(max(Num,P,Cls))     --> ['('], swrl_property(P), ['max'], swrl_number(Num), swrl_class_atom_terminal(Cls), [')'].
swrl_class_atom(min(Num,P,Cls))     --> ['('], swrl_property(P), ['min'], swrl_number(Num), swrl_class_atom_terminal(Cls), [')'].
swrl_class_atom(Cls)                --> swrl_class_atom_terminal(Cls).
swrl_class_atom_terminal(Cls)       --> [Cls_name], { swrl_match_instance(Cls, Cls_name, owl:'Class') }.

swrl_class_intersection([Cls])      --> swrl_class_atom(Cls).
swrl_class_intersection([Cls|Rest]) --> swrl_class_atom(Cls), ['and'], swrl_class_intersection(Rest).
swrl_class_union([Cls])             --> swrl_class_atom(Cls).
swrl_class_union([Cls|Rest])        --> swrl_class_atom(Cls), ['or'], swrl_class_union(Rest).

swrl_subject(S)    --> swrl_var_expr(S).
swrl_subject(S)    --> swrl_individual(S).
swrl_property(P)   --> [P_name], { swrl_match_instance(P, P_name, rdf:'Property') }.
swrl_object(Var)   --> swrl_var_expr(Var).
swrl_object(Obj)   --> swrl_individual(Obj).
swrl_object(Cls)   --> swrl_class_atom_terminal(Cls).

swrl_individual(I) --> [I_name], { swrl_match_instance(I, I_name, owl:'NamedIndividual') }.
swrl_number(Num)   --> [Num], { number(Num) }.

swrl_value(Val)   --> swrl_individual(Val).
swrl_value(Val)   --> swrl_class_atom_terminal(Val).
swrl_value(Val)   --> swrl_data_object(Val).

swrl_data_object(Var)                                                             --> swrl_var_expr(Var).
swrl_data_object(literal(type('http://www.w3.org/2001/XMLSchema#integer',Val)))   --> [Val], { integer(Val) }.
swrl_data_object(literal(type('http://www.w3.org/2001/XMLSchema#float',Val)))     --> [Val], { float(Val) }.
swrl_data_object(literal(type('http://www.w3.org/2001/XMLSchema#boolean',true)))  --> [true].
swrl_data_object(literal(type('http://www.w3.org/2001/XMLSchema#boolean',false))) --> [false].
swrl_data_object(literal(type('http://www.w3.org/2001/XMLSchema#string',Val)))    --> swrl_string_expr(Val).
swrl_data_object(literal(Val)) --> [Val], { atom(Val) }.

swrl_builtin_args([Arg|Rest]) --> swrl_builtin_arg(Arg), [','], swrl_builtin_args(Rest).
swrl_builtin_args([Arg])      --> swrl_builtin_arg(Arg).
swrl_builtin_arg(Var)         --> swrl_var_expr(Var).
swrl_builtin_arg(Val)         --> [Val], { number(Val) }.
swrl_builtin_arg(Val)         --> swrl_string_expr(Val).
swrl_builtin_arg(Atomic)      --> [Atomic], { atomic(Atomic) }.
swrl_builtin(Predicate)       --> [Predicate], { swrl_is_builtin(Predicate) }.

swrl_var_expr(var(Var)) --> ['?'], [Var].
swrl_string_expr(Val)   --> ['"'], [Val], ['"'].

swrl_is_builtin(Predicate) :-
  atom(Predicate), % check if predicate is a builtin predicate
  clause(swrl_builtin_pl(Term,_,_),_),
  Term =.. [Predicate|_].

swrl_match_instance(Iri,Name,Type) :-
  atom(Iri),
  rdfs_individual_of(Iri, Type),
  rdf_split_url(_, Name, Iri).
swrl_match_instance(Iri,Name,Type) :-
  var(Iri), atom(Name),
  rdf_current_prefix(_, Uri),
  rdf_split_url(Uri, Name, Iri),
  rdfs_individual_of(Iri, Type).
