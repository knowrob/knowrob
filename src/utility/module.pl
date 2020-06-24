:- module(utils_module,
        [ use_directory/1,
          interface/2
        ]).
/** <module> TODO

@author Daniel Beßler
@license BSD
*/

:- use_module('./filesystem.pl',
    [ path_concat/3
    ]).

:- dynamic interface_cache_/3.

%%
%
use_directory(Dir) :-
  path_concat(Dir,'__init__.pl',Path),
  user:consult(Path).

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
interface_load_(RelPath,Exports) :-
  atom(RelPath),
  prolog_load_context(directory,ModulePath),
  path_concat(ModulePath,RelPath,Path),
  interface_load1_(Path,Exports),
  !.
  
interface_load_(library(Lib),Exports) :-
  file_search_path(_,SearchPath),
  path_concat(SearchPath,Lib,Path),
  interface_load1_(Path,Exports),
  !.
  
interface_load_(RelPath,Exports) :-
  atom(RelPath),
  file_search_path(_,SearchPath),
  path_concat(SearchPath,RelPath,Path),
  interface_load1_(Path,Exports),
  !.
  
interface_load_(Path,Exports) :-
  atom(Path),
  interface_load1_(Path,Exports),
  !.

interface_load_(Path,_Exports) :-
  print_message(error, unresolved_interface(Path)),
  fail.

%%
interface_load1_(Path,Exports) :-
  atom(Path),
  \+ atom_concat(_,'.pl',Path),
  atom_concat(Path,'.pl',PathWithExtension),
  interface_load1_(PathWithExtension,Exports),
  !.

interface_load1_(Path,Exports) :-
  absolute_file_name(Path,AbsPath),
  exists_file(AbsPath),
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
  read_exports_(Exports,Xs,NeedsExpansion),
  ground(NeedsExpansion),
  % second: generate some expansions
  findall(Expansion, (
    expand_to_module_(Name,Xs,Expansion);
    expand_to_rdf_meta_(Name,Xs,Expansion);
    expand_to_rdfs_computable_(Name,Xs,Expansion)
  ), Expansions).

%%
expand_to_module_(Name,Exports,
        (:-module(Name,Pl_Predicates))) :-
  findall(Export, (
    member(X,Exports),
    ( X=[Functor,Arity|_] -> Export=(/(Functor,Arity)) ;
      X=op(_,_,_)         -> Export=X ;
      fail )
  ), Pl_Predicates).

%%
expand_to_rdf_meta_(Name,Exports,Expansion) :-
  findall(
    (:(Name,RDF_predicate)),
    ( member([Functor,_,Args|_],Exports),
      ground(Args),
      RDF_predicate=..[Functor|Args] ),
    RDF_List),
  argument_list_(RDF_List,RDF_Predicates),
  ( %Expansion=(:-use_module(library('semweb/rdf_db'), [rdf_meta/1]));
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
read_exports_([],[],_) :- !.
read_exports_([implements(I)|Xs],Y,true) :-
  interface_load_(I,Zs),
  read_exports_(Zs,Y0,_),
  read_exports_(Xs,Y1,_),
  append(Y0,Y1,Y).
read_exports_([X|Xs],[Y|Ys],Flag) :-
  read_export_(X,Y,Flag),
  read_exports_(Xs,Ys,Flag).

%%
read_export_((/(Functor,Arity)), [Functor,Arity,_,_],_) :- !.
read_export_(op(P,T,N), op(P,T,N),_) :- !.
read_export_((->(Term,Property)),
        [Functor,Arity,RDF_Args,Property],true) :-
  !, read_export_(Term,[Functor,Arity,RDF_Args,Property],_).
read_export_(     RDF_Predicate,
        [Functor,Arity,RDF_Args,_],true) :-
  compound(RDF_Predicate),
  RDF_Predicate=..[Functor|RDF_Args],
  % Test if the Args are the arguments for rdf_meta (see https://www.swi-prolog.org/pldoc/man?predicate=rdf_meta/1)
  subset(RDF_Args,[r,t,o,@,?,-,+,:]),
  length(RDF_Args,Arity),
  Arity>0.
read_export_(X,X,_) :- !.

%%
argument_list_([Xi], Xi).
argument_list_([X0,X1|Xs], ','(X0,Rest)) :-
  argument_list_([X1|Xs], Rest).
