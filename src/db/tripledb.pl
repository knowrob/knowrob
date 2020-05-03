:- module(tripledb,
    [ implements('itripledb'),
      tripledb_load/1,
      tripledb_load/2,
      tripledb_load/3,
      tripledb_tell(r,r,t,+),
      tripledb_ask(r,r,t,+,-),
      tripledb_ask(r,r,t,+),
      tripledb_ask(r,r,t),
      tripledb_forget(r,r,t,+),
      tripledb_forget(r,r,t),
      tripledb_subclass_of(r,r),     % ?Subclass, ?Class
      tripledb_subproperty_of(r,r),  % ?Subproperty, ?Property
      tripledb_subgraph_of/2,
      ros_package_iri/2
    ]).
/** <module> Interface for loading, storing, and retrieving triple data.

@author Moritz Tenorth
@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db.pl'), 
    [ load_rdf/3,
      rdf_register_ns/3
    ]).
:- use_module(library('http/http_open.pl'),
    [ http_open/3
    ]).

%ros_path/2
%ros_package_path/2

% get path from environment
tripledb_module(Module) :-
  getenv('PL_TRIPLE_STORE', Module),
  ( file_exists(Module) -> true ; (
    print_message(warning, tripledb(no_such_module(Module))),
    fail
  )),!.
% use fallback path
tripledb_module('db/mongo/tripledb').

%%
% Get path to module that implements the tripledb
% and import prefixed predicates from the module.
% TODO: rather do re-export? not sure if this kind of wrapping
%       adds much overhead.
:- tripledb_module(Module),
   use_module(Module,
        [ tripledb_init/0             as itripledb_init,
          tripledb_import/1           as itripledb_import,
          tripledb_export/1           as itripledb_export,
          tripledb_whipe/0            as itripledb_whipe,
          tripledb_load_rdf/3         as itripledb_load_rdf,
          tripledb_tell/5             as itripledb_tell,
          tripledb_ask/6              as itripledb_ask,
          tripledb_forget/5           as itripledb_forget,
          tripledb_subgraph_of/2      as itripledb_subgraph_of,
          tripledb_cache_get/2        as itripledb_cache_get,
          tripledb_cache_add/3        as itripledb_cache_add,
          tripledb_cache_invalidate/1 as itripledb_cache_invalidate
        ]).
:- dynamic ros_package_iri_/2.

%% ros_package_iri(+PkgName,+URI) is det.
%
% Register an IRI for a ROS package.
% When RDF files are loaded with the IRI prefix,
% it is first tried to serve the local file from
% the ROS package before downoading the file
% from the web.
%
ros_package_iri(PkgName,URI) :-
  assertz(ros_package_iri_(URI,PkgName)).

%% add default subgraph-of axioms
:- tripledb_subgraph_of(tbox,common).
:- tripledb_subgraph_of(user,tbox).

%% tripledb_load(+URL) is det.
%
% Parse a RDF file and load it into the tripledb.
% 
% Resolves owl:imports and supports both the common URL formats
% (file paths, file:// or http://) and the package:// URLs used
% in ROS to reference files with respect to the surrounding ROS
% package.
%
% @param URL Local or global file path or URL of the forms file://, http:// or package://
% 
tripledb_load(URL) :-
  tripledb_load(URL,[]).

%%
%
%
tripledb_load(URL,Args) :-
  is_list(Args),!,
  % register namespace
  ( member(namespace(NS),Args) ->
    ( atom_concat(URL,'#',Prefix),
      rdf_register_ns(NS,Prefix,[keep(true)]));
    ( true )
  ),
  % get graph name
  ( member(graph(Graph),Args) ->
    ( true );
    ( Graph=user )
  ),
  % get fact scope
  universal_scope(Scope),
  tripledb_load(URL,Scope,Graph).

%%
%
%
tripledb_load(URL,_,_) :-
  % TODO: replace old ontology version by new one
  tripledb_ask(URL,rdf:type,string(owl:'Ontology')),!.

tripledb_load(URL,Scope,Graph) :-
  tripledb_load1(URL,Triples),
  % load RDF data of imported ontologies
  forall(
    member(rdf(_,owl:'imports',I), Triples),
    tripledb_load(I,Scope,Graph)
  ),
  % load data into triple DB
  tripledb_load_rdf(Triples,Scope,Graph),
  % notify about asserted individuals
  forall(
    member(rdf(X,rdf:'type',owl:'NamedIndividual'), Triples),
    notify(individual(X))
  ).

%%
tripledb_load1(URL,Triples) :-
  sub_string(URL,0,4,_,'http'), !,
  http_open(URL,RDF_Stream,[]),
  tripledb_load2(URL,RDF_Stream,Triples),
  close(RDF_Stream).

tripledb_load1(URL,Triples) :-
  % map IRI to ROS package path based on predicate *ros_package_iri_*
  file_base_name(URL,FileName),
  file_directory_name(URL,Prefix),
  ros_package_iri_(Pkg,Prefix),
  ros_package_path(Pkg,PkgPath),
  % convention is that rdf files are stored in a directory named "owl" or "rdf"
  ( atomic_list_concat([PkgPath,owl,FileName],'/',LocalPath) ;
    atomic_list_concat([PkgPath,rdf,FileName],'/',LocalPath) ;
    atomic_list_concat([PkgPath,FileName],'/',LocalPath)
  ),
  exists_file(LocalPath),
  tripledb_load1(LocalPath,Triples),
  !.

tripledb_load1(URL,Triples) :-
  ros_path(URL,GlobalPath), !,
  tripledb_load1(GlobalPath,Triples).

tripledb_load1(URL,Triples) :-
  tripledb_load2(URL,URL,Triples).

tripledb_load2(Id,URL,Triples) :-
  % load rdf data. *Triples* is a ist of `rdf(Subject, Predicate, Object)` terms.
  load_rdf(URL, Triples, [blank_nodes(noshare)]).

		 /*******************************
		 *	   itripledb INTERFACE     	*
		 *******************************/

%% 
% @implements 'db/itripledb'
%
tripledb_init :- itripledb_init.

%% 
% @implements 'db/itripledb'
%
tripledb_import(Directory) :-
  itripledb_import(Directory).

%% 
% @implements 'db/itripledb'
%
tripledb_export(Directory) :-
  itripledb_export(Directory).

%% 
% @implements 'db/itripledb'
%
tripledb_whipe :-
  itripledb_whipe.

%% 
% @implements 'db/itripledb'
%
tripledb_load_rdf(RDF,Scope,Graph) :-
  itripledb_load_rdf(RDF,Scope,Graph).

%% 
% @implements 'db/itripledb'
%
tripledb_tell(S,P,update(O),Scope,Graph) :-
  time_scope_data(Scope,[Since,_]),!,
  tripledb_stop(S,P,Since),
  tripledb_tell1(S,P,O,Scope,Graph).

tripledb_tell(S,P,O,Scope,Graph) :-
  % HACK: in many cases convinient, but could cause issues
  %           to avoid adding more then one value to a functional property
  is_functional_property(P),!,
  time_scope_data(Scope,[Since,_]),
  tripledb_stop(S,P,Since),
  tripledb_tell1(S,P,O,Scope,Graph).

tripledb_tell(S,P,O,Scope,Graph) :-
  tripledb_tell1(S,P,O,Scope,Graph).

tripledb_tell1(S,P,O,Scope,Graph) :-
  itripledb_tell(S,P,O,Scope,Graph),
  ( is_symmetric_property(P) ->
    itripledb_tell(O,P,S,Scope);
    true
  ).

tripledb_tell(S,P,O,Scope) :-
  tripledb_tell(S,P,O,Scope,user).

%% 
% @implements 'db/itripledb'
%
tripledb_forget(S,P,O,Scope,Graph) :-
  itripledb_forget(S,P,O,Scope,Graph).

tripledb_forget(S,P,O,Scope) :-
  tripledb_forget(S,P,O,Scope,user).

tripledb_forget(S,P,O) :-
  tripledb_forget(S,P,O,_{},user).

%% 
% @implements 'db/itripledb'
%
tripledb_ask(S,P,O,QScope,FScope,Graph) :-
  itripledb_ask(S,P,O,QScope,FScope,Graph).

tripledb_ask(S,P,O,QScope,FScope) :-
  tripledb_ask(S,P,O,QScope,FScope,user).

tripledb_ask(S,P,O,QScope->FScope) :-
  tripledb_ask(S,P,O,QScope,FScope,user).

tripledb_ask(S,P,O) :-
  tripledb_ask(S,P,O,_{},_,user).

%%
%
tripledb_subclass_of(Class,Class).
tripledb_subclass_of(Subclass,Class) :-
  tripledb_ask(Subclass,rdfs:subClassOf,Class,_{},_,tbox).

%%
%
tripledb_subproperty_of(Property,Property).
tripledb_subproperty_of(Subproperty,Property) :-
  tripledb_ask(Subproperty,rdfs:subPropertyOf,Property,_{},_,tbox).

%% 
% @implements 'db/itripledb'
%
tripledb_cache_get(Query,Modules) :-
  itripledb_cache_get(Query,Modules).

%% 
% @implements 'db/itripledb'
%
tripledb_cache_add(Module,Predicate,Query) :-
  itripledb_cache_add(Module,Predicate,Query).

%% 
% @implements 'db/itripledb'
%
tripledb_cache_invalidate(Predicate) :-
  itripledb_cache_invalidate(Predicate).
