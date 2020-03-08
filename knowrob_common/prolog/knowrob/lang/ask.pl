
:- module('knowrob/lang/ask',
    [
      kb_triple/3,
      kb_triple/4,
      kb_classify/2,
      kb_classify/3,
      kb_type_of/2,
      kb_type_of/3,
      kb_some/3,
      property_range/3,
      property_cardinality/5
    ]).
/** <module> Some fundamental interfaces to use the KnowRob knowledge base.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/model/XSD')).
:- use_module(library('knowrob/model/Resource'), [
    kb_resource/1
]).
:- use_module(library('knowrob/comp/rdf_data'), [
    kb_rdf_pl/3
]).
:- use_module(library('knowrob/triples/triple_store'), [
    vkb_has_triple/4,
    vkb_has_type/3
]).

:- rdf_meta kb_triple(r,r,t),
            kb_triple(r,r,t,-),
            kb_classify(r,t),
            kb_classify(r,t,-),
            kb_type_of(r,t),
            kb_type_of(r,t,-),
            kb_some(r,r,r),
            property_range(r,t,r),
            property_cardinality(r,r,r,?,?).

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
property_range(Res,P,Range) :-
  findall(R, property_range_(Res,P,R), Ranges),
  owl_most_specific(Ranges,Range).

property_range_(Res,[Px],Range) :-
  % TODO use VKB
  owl_property_range_on_resource(Res, Px, Range), !.

property_range_(Res,[P1|Ps],Range) :-
  % TODO use VKB
  owl_property_range_on_resource(Res, P1, Y),!,
  property_range_(Y,Ps,Range).

property_range_(Res,P,Range) :-
  % TODO use VKB
  owl_property_range_on_resource(Res, P, Range).

%%
property_cardinality(Res,P,Range,Min,Max) :-
  % TODO use VKB
  % FIXME: there is a bug here that *some* restrictions are ignored
  owl_cardinality_on_resource(Res, P, Range, cardinality(Min,Max)).

%% kb_some(?Res,?P,?Range) is nondet.
%
% True if there must be at least one instance of Range linked
% with Res via the relation P.
%
kb_some(Res,P,Range) :-
  property_cardinality(Res,P,Range,Min,_),
  Min > 0.

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

		 /*******************************
		 *	HELPER			*
		 *******************************/

%%
is_most_specific(X,Xs) :-
  forall((
     member(Y,Xs),
     X \= Y
  ), \+ owl_subclass_of(Y, X)).
