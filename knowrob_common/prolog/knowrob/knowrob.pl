
:- module(knowrob,
    [
      kb_resource/1,
      %% ASK
      kb_triple/3,
      kb_triple/4,
      kb_classify/2,
      kb_classify/3,
      kb_type_of/2,
      kb_type_of/3,
      holds/1,         % ?Predicate(?Subject,?Object)
      holds/2,         % ?Predicate(?Subject,?Object), +Time
      holds/3,         % ?Subject, ?Predicate, ?Object
      holds/4,         % ?Subject, ?Predicate, ?Object, +Time
      property_range/3,
      property_cardinality/5,
      %%
      vkb_has_type/3,
      vkb_has_triple/4,
      %% TELL
      kb_create/2,
      kb_create/3,
      kb_assert/3,
      kb_assert/4,
      kb_retract/3,
      kb_retract/4,
      %% RDF
      kb_unique_id/2,
      kb_rdf_pl/3,
      kb_rdf_data/3,
      kb_rdf_object/2,
      kb_rdf_data_atom/2,
      kb_number_list/2,
      kb_reification/2,
      %%
      is_temporalized_property/1,
      set_temporalized_db/2
    ]).
/** <module> Some fundamental interfaces to use the KnowRob knowledge base.

@author Daniel Beßler
@license BSD
*/
% TODO switch to new RDF interface

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('knowrob/xsd')).

:- rdf_meta kb_resource(t),
            kb_create(r,-),
            kb_create(r,-,+),
            kb_assert(r,r,t),
            kb_assert(r,r,t,-),
            kb_retract(r,r,t),
            kb_retract(r,r,t,-),
            kb_triple(r,r,t),
            kb_triple(r,r,t,-),
            kb_classify(r,t),
            kb_classify(r,t,-),
            kb_type_of(r,t),
            kb_type_of(r,t,-),
            kb_reification(r,t),
            holds(t),
            holds(t,?),
            holds(r,r,t),
            holds(r,r,t,?),
            kb_rdf_pl(r,t,t),
            kb_rdf_data(r,t,t),
            kb_rdf_object(r,r),
            vkb_has_triple(r,r,t,-),
            vkb_has_type(r,r,-),
            is_temporalized_property(r),
            property_range(r,t,r),
            property_cardinality(r,r,r,?,?).

%%
:- dynamic temporalized_db/2.

%%
:- multifile vkb_has_triple/4,
             vkb_has_type/3,
             kb_rdf_data/3,
             kb_rdf_object/2.

%% kb_resource(+Res) is semidet.
%
% Test wether the resource provided in the argument
% is known in the knowledge base.
% A warning is printed if this is not the case.
%
kb_resource(Res) :- atom(Res), (
  rdf_has(Res,rdf:type,_);
  rdf_equal(Res,rdf:type)),!.

kb_resource(Res) :-
  print_message(warning, unknown_resource(Res)),
  fail.

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

		 /*******************************
		 *	TELL INTERFACE		*
		 *******************************/

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
		 *	ASK INTERFACE		*
		 *******************************/

%% kb_triple(?S,?P,?O) is nondet.
%% kb_triple(?S,?P,?O,+DBArgs) is nondet.
%
% True for triples (S,P,O) whose existence
% can be inferred by the knowledge base.
%
kb_triple(S,P,O) :-
  kb_triple(S,P,O,_{}).

kb_triple(S,P,O,DBArgs) :-
  vkb(DBArgs,DB),
  % input validation
  ( var(S);kb_resource(S) ),
  ( var(P);kb_resource(P) ),!,
  % try to instantiate O_rdf before calling owl_has
  ignore( kb_rdf_pl(P,O_rdf,O) ),
  % infer the triple
  ( ground([S,P,O]) ->
    once(owl_has(S,P,O_rdf,DB)) ; 
    owl_has(S,P,O_rdf,DB) ),
  % make sure O is grounded
  once(( ground(O) ; kb_rdf_pl(P,O_rdf,O) )).

%% kb_classify(+Entity,?Description) is nondet.
%% kb_classify(+Entity,?Description,+DBArgs) is nondet.
%
% Yields all (most specific) classifications of an
% entity that can be inferred by the knowledge base.
% Note that these may be restriction classes, union classes,
% etc. without direct super-class in the class taxonomy.
%
kb_classify(Entity,Description) :-
  kb_classify(Entity,Description,_{}).

kb_classify(Resource,Description,DBArgs) :-
  rdf_has(Resource, rdf:type, owl:'NamedIndividual'),!,
  %% classify Resource
  vkb(DBArgs,DB),
  ( ground(Description) -> (
  ( atom(Description) -> owl_individual_of(Resource,Description,DB) ; (
    % TODO: retract again
    owl_restriction_assert(Description,Id),
    owl_individual_of(Resource,Id,DB)
  ))) ; (
    setof(X, owl_individual_of(Resource,X,DB), Xs),
    member(Description,Xs),
    %% only select classes without sub-class in Xs
    is_most_specific(Description,Xs)
  )).

kb_classify(Resource,Type,_DBArgs) :-
  rdf_has(Resource,rdf:type,Type).

%% kb_type_of(+Entity,?Type) is nondet.
%% kb_type_of(+Entity,?Type,+DBArgs) is nondet.
%
% Yields all (most specific) types of an
% entity that can be inferred by the knowledge base.
%
kb_type_of(Entity,Type) :-
  kb_type_of(Entity,Type,_{}).

kb_type_of(Resource,Type,DBArgs) :-
  ground(Type), !,
  vkb(DBArgs,DB),
  ( ground(Resource) ->
  ( owl_individual_of(Resource,Type,DB), ! ) ;
    owl_individual_of(Resource,Type,DB)
  ).

kb_type_of(Resource,Type,DBArgs) :-
  ground(Resource),
  \+ ground(Type),
  findall(X, (
    vkb_has_triple(Resource,rdf:type,X,DBArgs),
    X \= 'http://www.w3.org/2002/07/owl#NamedIndividual'
  ), Xs),
  list_to_set(Xs,Set),
  member(Type,Set),
  %% only select classes without sub-class in Xs
  is_most_specific(Type,Set).

%%
property_range(Res,[Px],Range) :-
  % TODO use VKB
  owl_property_range_on_resource(Res, Px, Range), !.

property_range(Res,[P1|Ps],Range) :-
  % TODO use VKB
  owl_property_range_on_resource(Res, P1, Y),!,
  property_range(Y,Ps,Range).

property_range(Res,P,Range) :-
  % TODO use VKB
  owl_property_range_on_resource(Res, P, Range).

%%
property_cardinality(Res,P,Range,Min,Max) :-
  % TODO use VKB
  owl_cardinality_on_resource(Res, P, Range, cardinality(Min,Max)).

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
triple_db_has_property(S,P,O,_DBArgs) :-
  (  atom(O), rdf_has(P, rdf:type, owl:'DatatypeProperty') )
  -> rdf_has(S,P,literal(type(_,O)))
  ;  rdf_has(S,P,O).

%%
triple_db_has_type(Resource,Type,_DBArgs) :-
  rdfs_individual_of(Resource,Type).

%%
triple_db_no_during(DBArgs) :-
  ( \+ get_dict(during,DBArgs,_) ; (
    print_message(warning, triple_db(during_unsupported)),
    fail
  )),!.

		 /*******************************
		 *	Virtual KB 		*
		 *******************************/

%%
% The V(irtual)KB is a composition of
% backends for KnowRob.
% It is passed through the reasoning pipeline
% down to the DB access that is parametrized with
% the list of grounded arguments provided.
%
vkb(DBArgs, db(vkb_has_triple(DBArgs), vkb_has_type(DBArgs))).

%% vkb_has_triple(?S,?P,?O,+DBArgs) is nondet.
%
% This is a multifile predicate that can be extended
% in KnowRob packages to include additional knowledge sources.
%
vkb_has_triple(S,P,O,DBArgs) :-
  triple_db_has_property(S,P,O,DBArgs).

%% vkb_has_type(?S,?Type,+DBArgs) is nondet.
%
% This is a multifile predicate that can be extended
% in KnowRob packages to include additional knowledge sources.
%
vkb_has_type(S,Type,DBArgs) :-
  triple_db_has_type(S,Type,DBArgs).
  
vkb_has_type(S,Type,DBArgs) :-
  rdf_equal(rdf:type,Property),
  ( nonvar(Type) ->
    rdfs_subclass_of(SubType, Type) ;
    SubType = Type ),
  vkb_has_triple(S,Property,SubType,DBArgs).

		 /*******************************
		 *	holds predicate		*
		 *******************************/

%% holds(+Term).
%% holds(+Term, ?Interval).
%% holds(?S, ?P, ?O).
%% holds(?S, ?P, ?O, ?Interval).
%
% True iff relation `P(S,O)` holds during Interval, and with Term=P(S,O).
% Where Interval is a TimeInterval or TimePoint individual,
% a number or a list of two numbers representing a time interval.
%
% @param Term Must be of the form: PROPERTY(SUBJECT, OBJECT).
% @param T Can be TimeInterval or TimePoint individual, a number or a list of two numbers representing a time interval.
%
holds(Term) :-
  holds(Term,now).

holds(Term, Time) :-
  var(Term),
  holds(S,P,O,Time),
  Term =.. [':', P, (S,O)].

holds(Term, Time) :-
  nonvar(Term),
  unwrap_triple(Term,S,P,O),
  holds(S,P,O,Time).

holds(S,P,O) :-
  holds(S,P,O,now).

holds(S,P,O,now) :-
  !, kb_triple(S,P,O,_{}).

holds(S,P,O,Time) :-
  kb_triple(S,P,O,_{during:Time}).

%%
unwrap_triple(Term,S,P,O) :-
  Term =.. [':', Namespace, Tail],!,
  Tail =.. [PX,S,O],
  % unpack namespace
  rdf_current_ns(Namespace, NamespaceUri),
  atom_concat(NamespaceUri, PX, P).

unwrap_triple(Term,S,P,O) :-
  Term =.. [P,S,O].

		 /*******************************
		 *	RDF			*
		 *******************************/

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

%% kb_rdf_pl(?Property,?Obj_rdf,?Obj_pl) is semidet.
%
%
kb_rdf_pl(_,Obj_rdf,Obj_pl) :-
  %% both vars, nothing to do
  var(Obj_rdf), var(Obj_pl), !.

kb_rdf_pl(_,Obj_rdf,Obj_pl) :-
  %% Obj_rdf is bound to the Prolog value already
  ground(Obj_rdf),
  \+ atom(Obj_rdf),
  \+ Obj_rdf = literal(_),
  Obj_pl = Obj_rdf, !.

kb_rdf_pl(Property,Obj_rdf,Obj_pl) :-
  %% handle object properties
  ground(Property),
  rdf_has(Property, rdf:type, owl:'ObjectProperty'),
  % first try to use a conversion method.
  % This is used e.g. to support a prolog based representation
  % of transforms.
  ( kb_rdf_object(Obj_rdf,Obj_pl) ;
  % then try to unify plain atoms
  ( (atom(Obj_rdf);atom(Obj_pl)), Obj_pl = Obj_rdf ) ;
  % finally enforce atom by calling term_to_atom
  ( ground(Obj_pl), term_to_atom(Obj_pl,Obj_rdf),
    print_message(warning, kb_rdf_object(conversion_unknown(Property,Obj_pl)))
  )), !.

kb_rdf_pl(Property,Data_rdf,Data_pl) :-
  %% handle data properties
  (( ground(Property), rdf_has(Property, rdf:type, owl:'DatatypeProperty') ) ;
   ( nonvar(Data_rdf), Data_rdf=literal(_) ) ;
   ( nonvar(Data_pl), Data_pl=literal(Data_pl_x),
     ignore(Data_pl_x = type(Data_type,_)) )
  ),
  % instantiate Data_rdf in case it is a Var
  ( nonvar(Data_rdf) ;
    Data_rdf=literal(type(Data_type,Data_atom))
  ),
  % get the data atom and (optionally) the data type
  kb_rdf_data_atom(Data_rdf,Data_atom),
  ignore( kb_rdf_data_type(Property,Data_rdf,Data_type) ),
  % first try to use a conversion method.
  ( kb_rdf_data(Data_atom,Data_type,Data_pl) ; 
  % then try to unify plain atoms
  ( (atom(Data_atom);atom(Data_pl)), Data_pl = Data_atom ) ;
  % finally enforce atom by calling term_to_atom
  ( ground(Data_pl), term_to_atom(Data_pl,Data_atom),
    print_message(warning, kb_rdf_data(conversion_unknown(Property,Data_pl,Data_type)))
  )), !.

kb_rdf_pl(Property,Obj_rdf,Obj_pl) :-
  %% handle unknown properties
  \+ ground(Property),
  ( atom(Obj_rdf) ; atom(Obj_pl) ),
  Obj_pl = Obj_rdf, !.

%%
%kb_rdf_object(Obj_rdf,Obj_pl) :-
  %atom(Obj_rdf),
  %rdfs_individual_of(Obj_rdf,dul:'Collection'),!,
  %findall(X_pl, (
    %rdf_has(Obj_rdf,dul:hasMember,X_rdf),
    %kb_rdf_object(X_rdf,X_pl)),
    %Obj_pl
  %).

kb_rdf_object(Arg_rdf,Arg_pl) :-
  atom(Arg_rdf),
  rdfs_individual_of(Arg_rdf,ease:'Reification'),!,
  kb_reification(Arg_pl,Arg_rdf).

%%

kb_rdf_data(Data_atom,Data_type,Data_pl) :-
  nonvar(Data_pl),
  Data_pl = literal(X),
  ( X=type(Data_type,Data_atom) ;
    X=Data_atom
  ),!.

kb_rdf_data(Data_atom,Data_type,Data_pl) :-
  atom(Data_type),
  xsd_number_type(Data_type),
  atom_number(Data_atom,Data_pl),!.

kb_rdf_data(Data_string,Data_type,Data_atom) :-
  atom(Data_type),
  xsd_string_type(Data_type),
  string(Data_string),
  string_to_atom(Data_string,Data_atom), !.

kb_rdf_data(Data_atom,_Data_type,Data_pl) :-
  is_list(Data_pl),
  kb_number_list(Data_atom,Data_pl),!.

kb_rdf_data(Data_atom,Data_type,Data_pl) :-
  atom(Data_type),
  kb_number_list_type(Data_type),
  kb_number_list(Data_atom,Data_pl),!.

%%
kb_rdf_data_atom(literal(type(_,Atom)),Atom) :- !.
kb_rdf_data_atom(literal(Atom),Atom) :- !.
kb_rdf_data_atom(Atom,Atom) :- !.

%%
kb_rdf_data_type(_P,literal(type(Type,_)),Type) :-
  ground(Type), !.

kb_rdf_data_type(P, Data_rdf, Type) :-
  ground(P),
  rdf_phas(P, rdfs:range, Type),
  ignore(Data_rdf = literal(type(Type,_))), !.

%%
kb_number_list_type('http://www.ease-crc.org/ont/EASE.owl#array_boolean').
kb_number_list_type('http://www.ease-crc.org/ont/EASE.owl#array_double').
kb_number_list_type('http://www.ease-crc.org/ont/EASE.owl#array_float').
kb_number_list_type('http://www.ease-crc.org/ont/EASE.owl#array_int').
kb_number_list_type('http://www.ease-crc.org/ont/EASE.owl#array_uint').
kb_number_list_type('http://www.ease-crc.org/ont/EASE.owl#array_string').

%%
kb_number_list(List_atom,List_pl) :-
  ground(List_atom),!,
  atomic_list_concat(Atoms, ' ', List_atom),
  maplist(atom_number, Atoms, List_pl).

kb_number_list(List_atom,List_pl) :-
  ground(List_pl),!,
  kb_rdf_data_atom(List_pl, List),
  maplist(term_to_atom, List, Atoms),
  atomic_list_concat(Atoms, ' ', List_atom).

%%
kb_reification(Resource,Reification) :-
  kb_triple(Reification,ease:isReificationOf,Resource),!.

kb_reification(Resource,Reification) :-
  atom(Resource),
  kb_create(ease:'Reification',Reification),
  kb_assert(Reification,ease:isReificationOf,Resource).
  

		 /*******************************
		 *	HELPER			*
		 *******************************/

is_object_property(P,_O) :-
  atom(P),
  rdf_has(P, rdf:type, owl:'ObjectProperty'),!.

is_object_property(_P,O) :-
  atom(O),
  rdf_has(O, rdf:type, owl:'NamedIndividual'),!.

%%
is_temporalized_property(P) :-
  rdfs_subproperty_of(P,dul:hasRegion).

%%
is_most_specific(X,Xs) :-
  forall((
     member(Y,Xs),
     X \= Y
  ), \+ owl_subclass_of(Y, X)).
