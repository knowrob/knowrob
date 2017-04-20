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
      swrl_vars/2,
      swrl_var/3,
      swrl_var_bindings/2,
      swrl_holds/1,
      swrl_holds/2,
      swrl_atom_holds/2,
      swrl_project/1,
      swrl_print/1
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% RDF-based SWRL representation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% TODO(DB): Also allow to assert OWL descriptions from SWRl expressions / SWRL Prolog terms.

%% rdf_swrl_rule(?Descr, rule(?Head,?Body,?Vars))
%
% @Descr The RDF description of a SWRL rule
% @Head List of Prolog-encoded SWRL atoms
% @Body List of Prolog-encoded SWRL atoms
% @Vars Mapping between SWRL variable names and Prolog variables
%
rdf_swrl_rule(Descr, rule(Head,Body,Vars)) :-
  rdf_has(Descr, rdf:type, swrl:'Imp'),
  rdf_has(Descr, swrl:body, BodyDescr),
  rdf_has(Descr, swrl:head, HeadDescr),
  rdf_swrl_atom(BodyDescr, Body),
  rdf_swrl_atom(HeadDescr, Head),
  swrl_vars(rule(Head,Body), Vars).

%% rdf_swrl_atom(?Descr, ?List)
%% rdf_swrl_atom(?Descr, class(?Cls,?S))
%% rdf_swrl_atom(?Descr, restriction(?Restr,?S))
%% rdf_swrl_atom(?Descr, property(?S,?P,?O))
%
% Used for mapping RDF descriptions of SWRL rules to
% Prolog-encoded representation.
% Can either be used for querying the RDF description that
% contains particular classes, restrictions, and properties,
% or to generate a Prolog list representation of the RDF description.
%
% @Descr The RDF description of a SWRL rule
% @List List of Prolog-encoded SWRL atoms
% @Class Class IRI
% @Restr Restriction IRI
% @S Subject IRI or RDF identifier of SWRL variable
% @O Object IRI or RDF identifier of SWRL variable
% @P Property IRI
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

rdf_swrl_atom(Descr, not(class(Cls,S))) :-
  rdf_has(Descr, rdf:type, swrl:'ClassAtom'),
  rdf_has(Descr, swrl:classPredicate, ClsDescr),
  rdf_has(ClsDescr, owl:complementOf, ComplementDescr), !,
  rdf_swrl_atom(ComplementDescr,Cls),
  rdf_has(Descr, swrl:argument1, S).

rdf_swrl_atom(Descr, restriction(Cls,S)) :-
  rdf_has(Descr, rdf:type, swrl:'ClassAtom'),
  rdf_has(Descr, swrl:classPredicate, Cls),
  rdf_has(Cls, rdf:type, owl:'Restriction'), !,
  rdf_has(Descr, swrl:argument1, S).

rdf_swrl_atom(Descr, class(Cls,S)) :-
  rdf_has(Descr, rdf:type, swrl:'ClassAtom'),
  rdf_has(Descr, swrl:classPredicate, Cls),
  rdf_has(Descr, swrl:argument1, S).

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Variable mapping
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% swrl_vars(rule(+Head,+Body), ?Variables)
%
% @Head List of Prolog-encoded SWRL atoms
% @Body List of Prolog-encoded SWRL atoms
% @Variables Mapping between SWRL variable names and Prolog variables
%
swrl_vars(rule(Head,Body), Variables) :-
  findall(VarName, (
    (member(Atom,Head) ; member(Atom,Body)),
    swrl_var_names(Atom,VarName)
  ), VarNames),
  list_to_set(VarNames, VarNamesUnique),
  swrl_vars_pl(VarNamesUnique,Variables).

swrl_vars_pl([], []).
swrl_vars_pl([Name|RestNames], [var(Name,_)|RestVars]) :-
  swrl_vars_pl(RestNames, RestVars).

swrl_var_names(class(_,S), S) :-
  rdf_has(S, rdf:type, swrl:'Variable').
swrl_var_names(restriction(_,S), S) :-
  rdf_has(S, rdf:type, swrl:'Variable').
swrl_var_names(property(S,_,_), S) :-
  rdf_has(S, rdf:type, swrl:'Variable').
swrl_var_names(property(_,_,O), O) :-
  rdf_has(O, rdf:type, swrl:'Variable').
swrl_var_names(var(Var), Var).


%% swrl_var(+Vars, +Name, -PrologVar)
%
% @Vars Mapping between SWRL variable names and Prolog variables
% @Name A SWRL variable name
% @PrologVar The mapped Prolog variable
%
% TODO: SWI Prolog 7 has dicts: http://www.swi-prolog.org/pldoc/man?section=dicts
%
swrl_var(_, Name, Name) :-
  \+ rdf_has(Name, rdf:type, swrl:'Variable').
swrl_var(Vars, Name, PrologVar) :-
  swrl_var_(Vars, Name, PrologVar).

swrl_var_bindings(_,[]).
swrl_var_bindings(Vars,[[Name,Val]|Rest]) :-
  swrl_var(Vars, Name, Val),
  swrl_var_bindings(Vars,Rest).

swrl_var_([var(Name,PrologVar)|_], Name, PrologVar).
swrl_var_([var(Iri,PrologVar)|_], Name, PrologVar) :-
  rdf_split_url(_, Name, Iri).  
swrl_var_([var(_,_)|Rest], Name, PrologVar) :-
  swrl_var_(Rest, Name, PrologVar).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Atom checking
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

swrl_holds(Descr) :-
  atom(Descr),
  rdf_has(Descr, rdf:type, swrl:'Imp'),
  rdf_swrl_rule(Descr, Rule),
  swrl_holds(Rule).

swrl_holds(rule(_,Body,Vars)) :-
  swrl_holds(Body,Vars).

swrl_holds([], _).
swrl_holds([Atom|Rest], Vars) :-
  swrl_atom_holds(Atom,Vars),
  swrl_holds(Rest,Vars).

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
  atom(A), !, atom_number(A, A_num).
swrl_atom_number(var(A), A_num, Vars) :-
  !, swrl_var(Vars, A, A_var),
  ( ground(A_var) ->
    swrl_atom_number(A_var, A_num, Vars)
  ; A_num = A_var ).
swrl_atom_number(A, A_num, _) :-
  atom(A), !, atom_number(A, A_num).
swrl_atom_number(A_num, A_num, _) :- number(A_num), !.
swrl_nums([],[],_).
swrl_nums([X|Xs],[Y|Ys],Vars) :-
  swrl_atom_number(X,Y,Vars),
  swrl_nums(Xs,Ys,Vars).

%% Class atoms
swrl_atom_holds(class(Cls,S), Vars) :-
  atom(S),
  swrl_var(Vars, S, S_var),
  owl_individual_of(S_var, Cls).
swrl_atom_holds(not(Atom), Vars) :-
  not( swrl_atom_holds(Atom, Vars) ).
%% Restriction class atoms
swrl_atom_holds(restriction(Restr,S), Vars) :-
  atom(S),
  swrl_var(Vars, S, S_var),
  owl_satisfies_restriction(S_var, Restr).
%% Property atoms
swrl_atom_holds(property(S,P,O), Vars) :-
  %strip_literal_type(O, O_val),
  swrl_var(Vars, S, S_var),
  swrl_var(Vars, O, O_var),
  holds(S_var,P,O_var).
%% SWRL comparison builtins
swrl_atom_holds(equal(S,O), Vars) :-
  strip_literal_type(S, S_val),
  strip_literal_type(O, O_val),
  swrl_var(Vars, S_val, S_var),
  swrl_var(Vars, O_val, O_var),
  S_var = O_var.
swrl_atom_holds(notEqual(S,O), Vars) :-
  strip_literal_type(S, S_val),
  strip_literal_type(O, O_val),
  swrl_var(Vars, S_val, S_var),
  swrl_var(Vars, O_val, O_var),
  S_var \= O_var.
swrl_atom_holds(lessThan(A,B), Vars) :-
  swrl_nums([A,B], [A_num,B_num], Vars),
  A_num < B_num.
swrl_atom_holds(lessThanOrEqual(A,B), Vars) :-
  swrl_nums([A,B], [A_num,B_num], Vars),
  A_num =< B_num.
swrl_atom_holds(greaterThan(A,B), Vars) :-
  swrl_nums([A,B], [A_num,B_num], Vars),
  A_num > B_num.
swrl_atom_holds(greaterThanOrEqual(A,B), Vars) :-
  swrl_nums([A,B], [A_num,B_num], Vars),
  A_num >= B_num.
%% SWRL math builtins
swrl_atom_holds(add(A,B,C), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars),
  A_num is B_num + C_num.
swrl_atom_holds(subtract(A,B,C), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars),
  A_num is B_num - C_num.
swrl_atom_holds(multiply(A,B,C), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars),
  A_num is B_num * C_num.
swrl_atom_holds(divide(A,B,C), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars),
  A_num is B_num / C_num.
swrl_atom_holds(mod(A,B,C), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars),
  A_num is mod(B_num, C_num).
swrl_atom_holds(pow(A,B,C), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars),
  A_num is B_num ** C_num.
swrl_atom_holds(abs(A,B), Vars) :-
  swrl_nums([A,B], [A_num,B_num], Vars),
  B_num is abs(A_num).
%% SWRL boolean builtins
swrl_atom_holds(booleanNot(literal(type(
  'http://www.w3.org/2001/XMLSchema#boolean', 'false'))), _).
swrl_atom_holds(booleanNot(S), Vars) :-
  ground(S),
  swrl_var(Vars, S, S_var),
  swrl_atom_holds(booleanNot(S_var), Vars).
%% SWRL string builtins
swrl_atom_holds(stringConcat(A,B,C), Vars) :-
  swrl_atoms([A,B,C], [A_atom,B_atom,C_atom], Vars),
  atom_concat(C_atom,A_atom,B_atom).
swrl_atom_holds(stringLength(A,L), Vars) :-
  swrl_atom(A,A_atom,Vars),
  swrl_atom_number(L,L_num,Vars),
  atom_length(A_atom, L_num).
swrl_atom_holds(upperCase(A,Upper), Vars) :-
  swrl_atoms([A,Upper], [A_atom,Upper_atom], Vars),
  upcase_atom(A_atom, Upper_atom).
swrl_atom_holds(lowerCase(A,Lower), Vars) :-
  swrl_atoms([A,Lower], [A_atom,Lower_atom], Vars),
  upcase_atom(A_atom, Lower_atom).
swrl_atom_holds(contains(A,X), Vars) :-
  swrl_atoms([A,X], [A_atom,X_atom], Vars),
  sub_atom(A_atom, _, _, _, X_atom).
swrl_atom_holds(startsWith(A,X), Vars) :-
  swrl_atoms([A,X], [A_atom,X_atom], Vars),
  atom_concat(X_atom, _, A_atom).
swrl_atom_holds(endsWith(A,X), Vars) :-
  swrl_atoms([A,X], [A_atom,X_atom], Vars),
  atom_concat(_, X_atom, A_atom).
%swrl_atom_holds(matches(A,X), _) :-fail.
%% TODO: SWRL list builtins. Find example rule!
%listConcat, member, length, first, rest, empty

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% Atom projection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

swrl_project(Descr) :-
  atom(Descr),
  rdf_has(Descr, rdf:type, swrl:'Imp'),
  rdf_swrl_rule(Descr,Rule),
  swrl_holds(Rule),
  swrl_project(Rule).

swrl_project(rule(Head,_,Vars)) :-
  forall(
    member(Atom,Head),
    ( swrl_atom_holds(Atom,Vars) ;
      swrl_atom_project(Atom,Vars) )
  ).

%% Class atoms
% TODO: special treatment of destroyed class (end all SpatialThing types), or require statement in rule?
swrl_atom_project(class(Cls,S), Vars) :-
  swrl_var(Vars, S, S_var),
  assert_temporal_part(S_var, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', nontemporal(Cls)).
%% Property atoms
swrl_atom_project(property(S,P,O), Vars) :-
  strip_literal_type(O, O_val),
  swrl_var(Vars, S, S_var),
  swrl_var(Vars, O_val, O_var),
  assert_temporal_part(S_var, P, O_var).
%% Restriction class atoms
swrl_atom_project(restriction(Restr,S), Vars) :-
  rdf_has(Restr, rdf:type, owl:'Restriction'),
  rdf_has(Restr, owl:onProperty, P),
  swrl_var(Vars, S, S_var),
  swrl_project_restriction(S_var, P, Restr).
%% Builtins
swrl_atom_project(Term, _) :-
  write('WARN: Unable to project SWRL builtin '''),
  write(Term),
  writeln('''. builtin''s should not be used in rule heads.').

%% Restriction class atoms

swrl_project_restriction(S, P, Restr) :-
  % Retract all values that are not an instance of Cls
  rdf_has(Restr, owl:allValuesFrom, Cls),!,
  findall(O, (
      owl_has(S,P,O),
      \+ owl_individual_of(O,Cls)
  ), Os),
  list_to_set(Os,Values),
  forall( member(O,Values),
          assert_temporal_part_end(S,P,O) ).

swrl_project_restriction(S, P, Restr) :-
  % Assert random object fulfilling the relation
  rdf_has(Restr, owl:someValuesFrom, Cls),!,
  swrl_assert_random(S, P, Cls, 1).

swrl_project_restriction(S, P, Restr) :-
  % Retract all facts about objects fulfilling the relation
  rdf_has(Restr, owl:qualifiedCardinality, literal(type(_,0))), !,
  rdf_has(Restr, owl:onClass, Cls),
  swrl_relation_values(S, P, Cls, Os),
  forall( member(O,Os),
          assert_temporal_part_end(S,P,O) ).

swrl_project_restriction(S, P, Restr) :-
  % End or begin some random temporal parts with temporal relation P so that the number of
  % active temporal parts is equal to qualified cardinality.
  rdf_has(Restr, owl:qualifiedCardinality, literal(type(_,N))), !,
  rdf_has(Restr, owl:onClass, Cls),
  swrl_cardinality_diff(S, P, Cls, N, Diff),
  ( Diff >= 0 ->
    swrl_retract_random(S, P, Cls, Diff) ;
    swrl_assert_random(S, P, Cls, Diff) ).

swrl_project_restriction(S, P, Restr) :-
  % End some random temporal parts with temporal relation P so that the number of
  % active temporal parts is below specified max cardinality.
  rdf_has(Restr, owl:maxQualifiedCardinality, literal(type(_,N))),!,
  rdf_has(Restr, owl:onClass, Cls),
  swrl_cardinality_diff(S, P, Cls, N, Diff),
  swrl_retract_random(S, P, Cls, Diff).

swrl_project_restriction(S, P, Restr) :-
  rdf_has(Restr, owl:minQualifiedCardinality, literal(type(_,N))),!,
  rdf_has(Restr, owl:onClass, Cls),
  swrl_cardinality_diff(S, P, Cls, N, Diff),
  swrl_assert_random(S, P, Cls, Diff).

%% retract random objects that fulfills relation
swrl_retract_random(_, _, _, N) :- N =< 0, !.
swrl_retract_random(S, P, Cls, N) :-
  once((
    current_temporal_part(S,TemporalPart),
    rdf_has(TemporalPart, knowrob:'temporalProperty', P),
    temporal_part_value(TemporalPart, P, O),
    atom(O),
    owl_individual_of(O,Cls)
  )),
  M is N - 1, swrl_retract_random(S, P, Cls, M).

%% assert random objects that fulfills relation
swrl_assert_random(_, _, _, N) :- N >= 0, !.
swrl_assert_random(S, P, Cls, N) :-
  owl_individual_of(O,Cls),
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
      owl_individual_of(O,Cls)
  ), Os),
  list_to_set(Os,Values).

%% compute actual cardinality of a relation
swrl_relation_count(S,P,Cls,Count) :-
  swrl_relation_values(S,P,Cls,Values),
  length(Values, Count).

swrl_cardinality_diff(S, P, Cls, Num, Diff) :-
  swrl_relation_count(S, P, Cls, CurrentNum),
  Diff is CurrentNum - Num.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%% SWRL formatting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% TODO: come up with something bidirectional! Would like to generate terms from SWRL expressions.
%  swrl_format(Term, 'Men(?x) -> Person(?x)').
%  ->  Term = rule( class(..), class(...) )

swrl_print(rule(Head,Body,_)) :-
  swrl_print(rule(Head,Body)), write('\n').
swrl_print(rule(Head,Body)) :-
  swrl_print(Body),
  write(' -> '),
  swrl_print(Head). 
swrl_print([]) :- !.
swrl_print([Atom|Rest]) :- !,
  swrl_print(Atom),
  ignore(( Rest\=[], write(', '), swrl_print(Rest) )).
swrl_print(not(Cls,S)) :- !,
  write('(not ('),
  swrl_print(Cls),
  write('))('),
  swrl_print_atom(S),
  write(')').
swrl_print(class(Cls,S)) :- !,
  swrl_print_atom(Cls),
  write('('),
  swrl_print_atom(S),
  write(')').
swrl_print(restriction(Cls,S)) :- !,
  swrl_print_atom(Cls),
  write('('),
  swrl_print_atom(S),
  write(')').
swrl_print(literal(type(_,Atom))) :- write(Atom), !.
swrl_print(literal(Atom)) :- write(Atom), !.
swrl_print(property(S,P,literal(type(_,O)))) :- !,
  swrl_print_atom(P), write('('),
  swrl_print_atom(S),
  write(', '),
  write(O),
  write(')').
swrl_print(property(S,P,literal(O))) :- !, 
  swrl_print_atom(P), write('('),
  swrl_print_atom(S),
  write(', '),
  write(O),
  write(')').
swrl_print(property(S,P,O)) :- !, 
  swrl_print_atom(P), write('('),
  swrl_print_atom(S),
  write(', '),
  swrl_print_atom(O),
  write(')').
swrl_print(var(Term)) :- swrl_print(Term).
swrl_print(Term) :-
  compound(Term),
  Term =.. [Predicate|Atoms],
  write(Predicate), write('('),
  swrl_print(Atoms),
  write(')').
swrl_print(Descr) :-
  rdf_has(Descr, rdf:type, rdf:'List'),
  rdf_swrl_atom_list(Descr, List),
  swrl_print(List).
swrl_print(Atom) :-
  swrl_print_atom(Atom).

swrl_print_atom(Union) :-
  atom(Union),
  rdf_has(Union, owl:unionOf, Descr), !,
  write('('),
  rdf_swrl_atom_list(Descr, List),
  swrl_print_one_of(List),
  write(')').
swrl_print_atom(Atom) :-
  atom(Atom),
  rdf_has(Atom, owl:oneOf, Descr), !,
  write('{'),
  swrl_print(Descr),
  write('}').
swrl_print_atom(Restr) :-
  atom(Restr),
  rdf_has(Restr, rdf:type, owl:'Restriction'),
  rdf_has(Restr, owl:onProperty, P),
  write('('),
  swrl_print(P), write(' '),
  swrl_print_property_restriction(Restr),
  write(')').
swrl_print_atom(Atom_iri) :-
  atom(Atom_iri),
  rdf_split_url(_, Atom, Atom_iri),
  ignore(( rdf_has(Atom_iri, rdf:type, swrl:'Variable'), write('?') )),
  write(Atom).

swrl_print_one_of([]) :- !.
swrl_print_one_of([Atom|Rest]) :- !,
  swrl_print(Atom),
  ignore(( Rest\=[], write(' or '), swrl_print_one_of(Rest) )).

swrl_print_property_restriction(Restr) :-
  rdf_has(Restr, owl:someValuesFrom, Cls),
  write('some '), swrl_print(Cls).
swrl_print_property_restriction(Restr) :-
  rdf_has(Restr, owl:allValuesFrom, Cls),
  write('all '), swrl_print(Cls).
swrl_print_property_restriction(Restr) :-
  rdf_has(Restr, owl:qualifiedCardinality, literal(type(_,Num))),
  rdf_has(Restr, owl:onClass, Cls),
  write('exactly '), write(Num), write(' '), swrl_print(Cls).
swrl_print_property_restriction(Restr) :-
  rdf_has(Restr, owl:minQualifiedCardinality, literal(type(_,Num))),
  rdf_has(Restr, owl:onClass, Cls),
  write('min '), write(Num), write(' '), swrl_print(Cls).
swrl_print_property_restriction(Restr) :-
  rdf_has(Restr, owl:maxQualifiedCardinality, literal(type(_,Num))),
  rdf_has(Restr, owl:onClass, Cls),
  write('max '), write(Num), write(' '), swrl_print(Cls).


