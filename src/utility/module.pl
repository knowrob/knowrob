:- module(utils_module,
        [ use_directory/1,
          strip_module/2,
          interface/2
        ]).
/** <module> TODO

@author Daniel Beßler
@license BSD
*/

:- use_module('./filesystem.pl'
    [ path_concat/3
    ]).

:- dynamic interface_cache_/3.

%%
%
use_directory(Dir) :-
  path_concat(Dir,'__init__.pl',Path),
  consult(Path).

%%
% Strip first argument of (:(_,_)) terms.
%
strip_module(:(_,Term),Term) :- !-
strip_module(Term,Term).

%% interface(+Name,+Exports) is det
%
% Defines a new interface. The syntax is equal to the one used to define 
% modules. Modules implementing the interface include a term
% *implements(InterfaceFile)* in the list of exported terms.
%
% file a:
%     :- interface(iface,
%             [ test(r,r) -> rdf:type
%             ]).
% file b:
%     :- module(iface_impl,
%             [ implements(iface),
%               test2(t),
%               test3/2
%             ]).
%
interface(Name,Exports) :-
  interface_cache_(Name,_,Exports), !.

interface(Name,Exports) :-
  % get path to file where interface is defined
  %  (it is the source file which is currently being loaded).
  prolog_load_context(source,Path),
  assertz( interface_cache_(Name,Path,Exports) ).

%%
% Read list of exports of an interface (not recursively).
%
interface_load_(Path,Exports) :-
  \+ atom_concat(_,'.pl',Path),!,
  atom_concat(Path,'.pl',PathWithExtension),
  interface_load_(PathWithExtension,Exports).

interface_load_(Path,Exports) :-
  absolute_file_name(Path,AbsPath),
  ensure_loaded(AbsPath),
  interface_cache_(_,AbsPath,Exports).

%%
% Expand *module* directives.
% This also extends the syntax of items in the 
% export list of modules. The may also include
% RDF meta information.
%
user:term_expansion((:-module(Name,Exports)), Expansions) :-
  % first: transform each exported element into common structure,
  %        and resolve *implements(_)* terms.
  read_exports_(Exports,Xs),
  % second: generate some expansions
  findall(Expansion, (
    expand_to_module_(Name,Xs,Expansion);
    expand_to_rdf_meta_(Name,Xs,Expansion);
    expand_to_rdfs_computable_(Name,Xs,Expansion)
  ), Expansions).

%%
expand_to_module_(Name,Exports,
        (:-module(Name,Pl_Predicates))) :-
  findall(
    (/(Functor,Arity)),
    member([Functor,Arity|_],Exports),
    Pl_Predicates).

%%
expand_to_rdf_meta_(Name,Exports,Expansion) :-
  findall(
    (:(Name,RDF_predicate)),
    ( member([Functor,_,Args|_],Exports),
      ground(Args),
      RDF_predicate=..[Functor|Args] ),
    RDF_List),
  argument_list_(RDF_List,RDF_Predicates),
  ( Expansion=(:-use_module(library('semweb/rdf_db'), [rdf_meta/1]));
    Expansion=(:-rdf_meta(RDF_Predicates))
  ).

%%
expand_to_rdfs_computable_(Name,Exports,Expansion) :-
  findall(
    (:(Name,Compute_Predicate)),
    ( member([Functor,_,_,Property|_],Exports),
      ground(Property),
      Compute_Predicate=..[Functor,Property]
    ),
    Compute_List),
  argument_list_(Compute_List,Compute_Predicates),
  ( Expansion=(:-use_module(library('lang/computable'), [computables/1]));
    Expansion=(:-computables(Compute_Predicates))
  ).

%%
read_exports_([],[]) :- !.
read_exports_([implements(I)|Xs],Y) :-
  interface_load_(I,Zs),
  read_exports_(Zs,Y0),
  read_exports_(Xs,Y1),
  append(Y0,Y1,Y).
read_exports_([X|Xs],[Y|Ys]) :-
  read_export_(X,Y),
  read_exports_(Xs,Ys).

%%
read_export_((/(Functor,Arity)),
        [Functor,Arity,_,_]) :-
  !.
read_export_((->(Term,Property)),
        [Functor,Arity,RDF_Args,Property]) :-
  !, read_export_(Term,[Functor,Arity,RDF_Args,Property]).
read_export_(     RDF_Predicate,
        [Functor,Arity,RDF_Args,_]) :-
  compound(RDF_Predicate),
  RDF_Predicate=..[Functor|RDF_Args],
  length(RDF_Args,Arity).

%%
argument_list_([Xi], Xi).
argument_list_([X0,X1|Xs], ','(X0,Rest)) :-
  argument_list_([X1|Xs], Rest).
