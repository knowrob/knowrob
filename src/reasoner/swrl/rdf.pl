
:- module(rdf_swrl,
    [
      rdf_swrl_rule/2,
      rdf_swrl_atom/2,
      rdf_swrl_load/0,
      rdf_swrl_load/1,
      rdf_swrl_unload/0,
      rdf_swrl_unload/1
    ]).
/** <module> RDF-based SWRL representation.

@author Daniel Beßler
*/

:- use_module(library('semweb'), [sw_register_prefix/2]).
:- sw_register_prefix(swrl, 'http://www.w3.org/2003/11/swrl#').
:- sw_register_prefix(swrla, 'http://swrl.stanford.edu/ontologies/3.3/swrla.owl#').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).

:- use_module('./swrl.pl').

:- dynamic  rdf_swrl_store/2.

%% rdf_swrl_rule(?Descr, ?Term).
%
% `Term` is a structure that represents the RDF SWRL rule `Descr`.
%
% @param Descr The RDF description of a SWRL rule
% @param Term Prolog term representing a SWRL rule
%
rdf_swrl_rule(Descr, Head :- Body) :-
  rdf_has(Descr, rdf:type, swrl:'Imp'),
  rdf_has(Descr, swrl:body, BodyDescr),
  rdf_has(Descr, swrl:head, HeadDescr),
  rdf_swrl_atom(BodyDescr, Body),
  rdf_swrl_atom(HeadDescr, Head).

rdf_swrl_description(Descr,Descr) :-
  atom(Descr), rdf_has(Descr, rdf:type, swrl:'Imp'), !.
rdf_swrl_description(Name,Descr) :-
  atom(Name), rdf_swrl_name(Descr,Name).

%% rdf_swrl_atom(?Descr, ?Term).
%
% `Term` is a structure that represents the RDF SWRL rule atom `Descr`.
%
% @param Descr The RDF description of a SWRL atom
% @param Term Prolog term representing a SWRL atom
%
rdf_swrl_atom(Descr, List) :-
  rdf_has(Descr, rdf:type, swrl:'AtomList'), !,
  rdf_swrl_atom_list(Descr, List).
rdf_swrl_atom(Descr, List) :-
  rdf_has(Descr, rdf:type, rdf:'List'), !,
  rdf_swrl_atom_list(Descr, List).

rdf_swrl_atom(literal(type(
  'http://www.w3.org/2001/XMLSchema#anyURI', Val)), Val) :- !.
rdf_swrl_atom(literal(type(Type, Val)),
              literal(type(Type, Val))).
rdf_swrl_atom(literal(Val), literal(Val)).

rdf_swrl_atom(Descr, var(Descr)) :-
  rdf_has(Descr, rdf:type, swrl:'Variable'), !.

rdf_swrl_atom(Cls, Cls) :-
  rdf_has(Cls, rdf:type, owl:'Class'), !.

rdf_swrl_atom(Descr, class(Cls,S)) :-
  rdf_has(Descr, rdf:type, swrl:'ClassAtom'), !,
  rdf_has(Descr, swrl:classPredicate, Cls),
  rdf_has(Descr, swrl:argument1, S).

rdf_swrl_atom(Descr, property(S,P,O)) :-
  rdf_has(Descr, rdf:type, swrl:'IndividualPropertyAtom'), !,
  rdf_has(Descr, swrl:propertyPredicate, P),
  rdf_has(Descr, swrl:argument1, S),
  rdf_has(Descr, swrl:argument2, O).

rdf_swrl_atom(Descr, property(S,P,O)) :-
  rdf_has(Descr, rdf:type, swrl:'DatavaluedPropertyAtom'), !,
  rdf_has(Descr, swrl:propertyPredicate, P),
  rdf_has(Descr, swrl:argument1, S),
  rdf_has(Descr, swrl:argument2, O).

rdf_swrl_atom(Descr, Atom) :-
  rdf_has(Descr, rdf:type, swrl:'BuiltinAtom'), !,
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

%% rdf_swrl_name(?Descr,?Name).
%
% `Name` is the rdfs:label value of the RDF SWRL rule `Descr`.
%
% @param Descr The RDF description of a SWRL rule
% @param Name The rdfs:label of the rule.
%
rdf_swrl_name(Descr, Name) :-
  rdf_has(Descr, rdfs:label, literal(type(_,Name))), 
  rdf_has(Descr, rdf:type, swrl:'Imp'), !.

%% rdf_swrl_enabled(?Descr).
%
% True if the RDF SWRL rule 'Descr' is enabled.
%
% @Descr The RDF description of a SWRL rule
%
rdf_swrl_enabled(Descr) :-
  rdf_has(Descr, swrla:isRuleEnabled, Val),
  ( Val=true ; Val=literal(true) ; Val=literal(type(_,true))), !.

%% rdf_swrl_load.
% 
% Asserts all enabled RDF SWRL rules in Prolog KB
%
rdf_swrl_load :- forall((
  rdf_has(Descr, rdf:type, swrl:'Imp'),
  rdf_swrl_enabled(Descr)),(
  rdf_swrl_rule(Descr, Rule),
  rdf_swrl_assert(Descr,Rule))).

%% rdf_swrl_load(+Descr).
% 
% Asserts RDF SWRL rule `Descr` in Prolog KB
%
% @param Descr RDF identifier of SWRl rule description, or the name of a rule.
%
rdf_swrl_load(Descr) :-
  rdf_swrl_description(Descr, Descr_),
  rdf_swrl_rule(Descr_, Rule),
  rdf_swrl_assert(Descr_, Rule).
 
rdf_swrl_assert(Descr,Rule) :-
  \+ rdf_swrl_store(Descr,_),
  swrl_assert(Rule, Rules),
  forall(member(R,Rules), assertz( rdf_swrl_store(Descr,R) )).

%% rdf_swrl_unload.
% 
% Retacts all previously loaded RDF SWRL rules from Prolog KB
%
rdf_swrl_unload :-
  forall( rdf_swrl_store(Descr, Rule), (
      retract( rdf_swrl_store(Descr,Rule) ),
      ignore(retract( Rule ))
  )).

%% rdf_swrl_unload(+Descr).
% 
% Retacts the RDF SWRL rule `Descr` from Prolog KB
%
% @param Descr RDF identifier of SWRl rule description, or the name of a rule.
%
rdf_swrl_unload(Descr) :-
  rdf_swrl_description(Descr, Descr_),
  rdf_swrl_retract(Descr_).
 
rdf_swrl_retract(Descr) :-
  forall( rdf_swrl_store(Descr, Rule), (
      retract( rdf_swrl_store(Descr,Rule) ),
      ignore(retract( Rule ))
  )).
