
:- module('knowrob/lang/tell',
    [
      kb_unique_id/2,
      kb_create/2,
      kb_create/3,
      kb_assert/3,
      kb_assert/4,
      kb_retract/3,
      kb_retract/4,
      %%
      is_temporalized_property/1,
      set_temporalized_db/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl'), [rdfs_list_to_prolog_list/2]).
:- use_module(library('knowrob/model/Resource'), [
    kb_resource/1
]).
:- use_module(library('knowrob/comp/rdf_data'), [
    kb_rdf_pl/3,
    kb_rdf_data_atom/2
]).

:- rdf_meta kb_create(r,-),
            kb_create(r,-,+),
            kb_assert(r,r,t),
            kb_assert(r,r,t,-),
            kb_retract(r,r,t),
            kb_retract(r,r,t,-),
            is_temporalized_property(r).

%%
:- dynamic temporalized_db/2.

%% set_temporalized_assert(+Ask,+Tell) is det.
%
% Used to configure the DB used for temporalized
% triples that have an additional time argument.
%
%
set_temporalized_db(Goal_assert,Goal_retract) :-
  retractall(temporalized_db(_,_)),
  assertz(temporalized_db(Goal_assert,Goal_retract)).
% initialize temporal db to triple db.
:- set_temporalized_db(triple_db_assert,triple_db_retract).

%%
is_temporalized_property(P) :-
  rdfs_subproperty_of(P,dul:hasRegion).

%% kb_unique_id(+Class, -UniqID) is det.
%
% UniqID is a IRI that uses Class as prefix and is
% not yet used in the RDF triple store.
%
% @param Class Class IRI
% @param UniqID Unused IRI with prefix Class
%
kb_unique_id(Class, UniqID) :-
  % generate 8 random alphabetic characters
  randseq(8, 25, Seq_random),
  maplist(plus(65), Seq_random, Alpha_random),
  atom_codes(Sub, Alpha_random),
  atom_concat(Class,  '_', Class2),
  atom_concat(Class2, Sub, Instance),
  % check if there is no triple with this identifier as subject or object yet
  ( rdf_has(Instance,_,_) ->
    kb_unique_id(Class, UniqID);
    UniqID = Instance
  ).

%% kb_create(+Type,-Instance) is semidet.
%% kb_create(+Type,-Instance,+DBArgs) is det.
%
% Instantiate an known class.
%
% @param Type The RDF name of the class.
% @param Instance The instance created.
% @param DBArgs DB arguments that contextualize the query.
%
kb_create(Type,Instance) :-
  kb_create(Type,Instance,_{}).

kb_create(Type,Instance,DBArgs) :-
  % create instance of intersection class
  rdf_has(Type,owl:intersectionOf,Set),
  rdfs_list_to_prolog_list(Set,Members),
  % find concept name
  member(AtomicType,Members),
  rdfs_subclass_of(AtomicType,dul:'Entity'),!,
  kb_create(AtomicType,Instance,DBArgs),
  % assert the other types
  forall(
    (member(X,Members), X \= AtomicType),
    kb_assert(Instance,rdf:type,X)
  ).

kb_create(Type,Instance,DBArgs) :-
  kb_resource(Type),
  triple_db_create(Type,Instance,DBArgs).

%% kb_assert(+S,+P,+O) is semidet.
%% kb_assert(+S,+P,+O,+DBArgs) is det.
%
% Assert a triple in the RDF store.
%
% @param S The subject of the triple.
% @param P The predicate of the triple.
% @param O The object of the triple.
% @param DBArgs DB arguments that contextualize the query.
%
kb_assert(S,P,O) :-
  kb_assert(S,P,O,_{}).

kb_assert(S,P,O,DBArgs) :-
  % input validation:
  % - P must be a known property
  once(( ground([S,P,O]) ; (
    print_message(warning, kb_assert(not_ground(S,P,O))),
    fail
  ))),
  kb_resource(P),
  % input conversion
  % - Prolog-encoded data to typed RDF literals
  once(( kb_rdf_pl(P,O_rdf,O) ; (
    atom(O), O_rdf=O
  ))),
  % TODO: auto-classify P?
  % TODO: auto-retract old value when functional?
  kb_assert_(S,P,O_rdf,DBArgs).

kb_assert_(S,P,O,DBArgs) :-
  % special handling for "temporalized" properties
  % that have an additional time argument.
  is_temporalized_property(P),!,
  temporalized_db(Goal,_),
  once(( call(Goal,S,P,O,DBArgs) ; (
    print_message(warning, temporalized_db(assert_failed)),
    fail
  ))).

kb_assert_(S,P,O,DBArgs) :-
  % assert to RDF triple store.
  triple_db_assert(S,P,O,DBArgs).

kb_retract(S,P,O) :-
  kb_retract(S,P,O,_{}).

kb_retract(S,P,O,DBArgs) :-
  is_temporalized_property(P),!,
  temporalized_db(_,Goal),
  ( call(Goal,S,P,O,DBArgs) ; (
    print_message(warning, temporalized_db(retract_failed)),
    fail
  )),!.

%%
kb_retract(S,P,O,DBArgs) :-
  % retract from RDF triple store.
  triple_db_retract(S,P,O,DBArgs),!.


		 /*******************************
		 *	TRIPLE DB		*
		 *******************************/

%% TODO: there are a couple of reified relations in dul.
%%         e.g. Classification, this should be included
%%         to infer triples and types.

%%
triple_db_create(Type,Instance,DBArgs) :-
  triple_db_no_during(DBArgs),
  ( get_dict(graph,DBArgs,G) ; G=belief_state ),
  kb_unique_id(Type,Instance), !,
  rdf_assert(Instance,rdf:type,Type,G),
  ( rdfs_individual_of(Type,owl:'Class') -> (
    rdf_assert(Instance,rdf:type,owl:'NamedIndividual',G)
  ) ; true ).

%%
triple_db_assert(S,P,O,DBArgs) :-
  triple_db_no_during(DBArgs),
  ( get_dict(graph,DBArgs,G) ;
    rdf(S,rdf:type,_,G) ;
    G = belief_state
  ),
  ( ground(O) -> O_ground=O ; (
  ( O=literal(_), 
    kb_rdf_data_atom(O,O_Atom),
    O_ground=literal(O_Atom)
  ))),!,
  rdf_assert(S,P,O_ground,G).

%%
triple_db_retract(S,P,O,_DBArgs) :-
  rdf_retractall(S,P,O).

%%
triple_db_no_during(DBArgs) :-
  ( \+ get_dict(during,DBArgs,_) ; (
    print_message(warning, triple_db(during_unsupported)),
    fail
  )),!.
