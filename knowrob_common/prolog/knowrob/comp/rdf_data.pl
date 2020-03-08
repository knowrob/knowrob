
:- module('knowrob/comp/rdf_data',
    [
      kb_rdf_pl/3,
      kb_rdf_data/3,
      kb_rdf_object/2,
      kb_rdf_data_atom/2,
      kb_number_list/2
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob/comp/tf'), [
    transform_reference_frame/2,
    transform_data/2
]).
:- use_module(library('knowrob/model/Reification'), [
    kb_reification/2
]).
:- use_module(library('knowrob/model/XSD'), [
    xsd_number_type/1,
    xsd_string_type/1
]).

:- rdf_meta kb_rdf_pl(r,t,t),
            kb_rdf_data(r,t,t),
            kb_rdf_object(r,r).

%%
:- multifile kb_rdf_data/3,
             kb_rdf_object/2.

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

kb_rdf_object(SpaceRegion,[Ref_frame,_,Pos,Rot]) :-
  atom(SpaceRegion),
  rdfs_individual_of(SpaceRegion,ease_obj:'6DPose'),!,
  transform_reference_frame(SpaceRegion,Ref_frame),
  transform_data(SpaceRegion,(Pos,Rot)).

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
% TODO move to model
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
