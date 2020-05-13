:- module(tripledb,
    [ implements('./itripledb.pl'),
      tripledb_load/1,
      tripledb_load/2,
      tripledb_load/3,
      tripledb_tell(r,r,t),
      tripledb_tell(r,r,t,+),
      tripledb_ask(r,r,t,+,-,+),
      tripledb_ask(r,r,t,+,-),
      tripledb_ask(r,r,t),
      tripledb_forget(r,r,t,+),
      tripledb_forget(r,r,t)
    ]).
/** <module> Interface for loading, storing, and retrieving triple data.

@author Daniel Beßler
@license BSD
*/

:- use_module(library(rdf),              [ load_rdf/3 ]).
:- use_module(library('semweb/rdf_db'),  [ rdf_equal/2, rdf_register_ns/3 ]).
:- use_module(library('http/http_open'), [ http_open/3 ]).

:- use_module(library('utility/url'),    [ url_resolve/2 ]).
:- use_module(library('db/scope'),       [ universal_scope/1 ]).
:- use_module(library('comm/notify'),    [ notify/1 ]).
:- use_module(library('model/XSD'),      [ xsd_data_basetype/2 ]).

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
   use_module(library(Module),
        [ tripledb_init/0             as itripledb_init,
          tripledb_import/1           as itripledb_import,
          tripledb_export/1           as itripledb_export,
          tripledb_whipe/0            as itripledb_whipe,
          tripledb_tell/5             as itripledb_tell,
          tripledb_bulk_tell/3        as itripledb_bulk_tell,
          tripledb_ask/6              as itripledb_ask,
          tripledb_forget/5           as itripledb_forget,
          tripledb_cache_get/3        as itripledb_cache_get,
          tripledb_cache_add/3        as itripledb_cache_add,
          tripledb_cache_invalidate/1 as itripledb_cache_invalidate
        ]).
% whipe the triple DB initially if requested by the user.
:- getenv('KB_WHIPE_TRIPLE_STORE',true) -> itripledb_whipe ; true.

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
  % FIXME: Make sure we do not load multiple versions of same ontology!
  once(( url_unresolve(URL,Unresolved) ; Unresolved=URL )),
  tripledb_ask(Unresolved,rdf:type,string(owl:'Ontology')),!.

tripledb_load(URL,Scope,Graph) :-
  rdf_equal(owl:'imports',OWL_Imports),
  rdf_equal(owl:'NamedIndividual',OWL_NamedIndividual),
  rdf_equal(rdf:'type',RDF_Type),
  %%
  tripledb_load1(URL,URL_resolved,Triples),
  % first, load RDF data of imported ontologies
  forall(
    member(rdf(_,OWL_Imports,I), Triples),
    tripledb_load(I,Scope,Graph)
  ),
  % load data into triple DB
  print_message(informational,tripledb(load(URL_resolved))),
  tripledb_load3t(Triples,Scope,Graph),
  % notify about asserted individuals
  forall(
    member(rdf(X,RDF_Type,OWL_NamedIndividual), Triples),
    notify(individual(X))
  ).

%%
tripledb_load1(URL,Resolved,Triples) :-
  % try to resolve to local path
  url_resolve(URL,Resolved),!,
  tripledb_load2(Resolved,Triples).

tripledb_load1(URL,URL,Triples) :-
  sub_string(URL,0,4,_,'http'), !,
  http_open(URL,RDF_Stream,[]),
  tripledb_load2(RDF_Stream,Triples),
  close(RDF_Stream).

tripledb_load1(URL,URL,Triples) :-
  tripledb_load2(URL,Triples).

tripledb_load2(URL,Triples) :-
  % load rdf data. *Triples* is a ist of `rdf(Subject, Predicate, Object)` terms.
  load_rdf(URL, Triples,[blank_nodes(noshare)]).

%%
tripledb_load3(Triples,Scope,Graph) :-
  findall(Converted, (
    member(Triple0,Triples),
    convert_rdf_(Triple0,Converted)
  ), ConvertedTriples),
  tripledb_bulk_tell(ConvertedTriples,Scope,[graph(Graph)]).

tripledb_load3t(Triples,Scope,Graph) :-
  % debug how long loading takes
  length(Triples,NumTriples),
  get_time(Time0),
  tripledb_load3(Triples,Scope,Graph),
  get_time(Time1),
  PerSec is NumTriples/(Time1-Time0),
  print_message(informational, tripledb(loaded(ntriples(NumTriples),persecond(PerSec)))).

%%
tripledb_load_rdf_(Triple,Scope,Options) :-
  convert_rdf_(Triple,rdf(S,P,O)) ->
    tripledb_tell(S,P,O,Scope,Options);
    true.

%%
convert_rdf_(rdf(_,P,_),_) :-
  ( rdf_equal(P,rdfs:comment)
  ; rdf_equal(P,rdfs:seeAlso)
  ; rdf_equal(P,owl:versionInfo) ),!,
  fail.

convert_rdf_(rdf(S,P,O),rdf(S,P,O1)) :-
  convert_rdf_value_(O,O1),
  !.

%%
convert_rdf_value_(literal(type(Type,O)),O_typed) :-
  xsd_data_basetype(Type,TypeKey),
  O_typed=..[TypeKey,O].
convert_rdf_value_(literal(O),string(O)).
convert_rdf_value_(O,string(O)).

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
tripledb_tell(S,P,update(O),Scope,Options) :-
  time_scope_data(Scope,[Since,_]),!,
  tripledb_stop(S,P,Since),
  itripledb_tell(S,P,O,Scope,Options).

tripledb_tell(S,P,O,Scope,Options) :-
  itripledb_tell(S,P,O,Scope,Options).

tripledb_tell(S,P,O,Scope) :-
  tripledb_tell(S,P,O,Scope,[]).

tripledb_tell(S,P,O) :-
  universal_scope(Scope),
  tripledb_tell(S,P,O,Scope,[]).

%% 
% @implements 'db/itripledb'
%
tripledb_bulk_tell(Facts,Scope,Options) :-
  itripledb_bulk_tell(Facts,Scope,Options).

%% 
% @implements 'db/itripledb'
%
tripledb_forget(S,P,O,Scope,Options) :-
  itripledb_forget(S,P,O,Scope,Options).

tripledb_forget(S,P,O,Scope) :-
  itripledb_forget(S,P,O,Scope,[]).

tripledb_forget(S,P,O) :-
  wildcard_scope(QScope),
  itripledb_forget(S,P,O,QScope,[]).

%% 
% @implements 'db/itripledb'
%
tripledb_ask(S,P,O,QScope,FScope,Options) :-
  itripledb_ask(S,P,O,QScope,FScope,Options).

tripledb_ask(S,P,O,QScope,FScope) :-
  itripledb_ask(S,P,O,QScope,FScope,[]).

tripledb_ask(S,P,O) :-
  wildcard_scope(QScope),
  itripledb_ask(S,P,O,QScope,_,[]).

%% 
% @implements 'db/itripledb'
%
tripledb_cache_get(Predicate,Query,Modules) :-
  itripledb_cache_get(Predicate,Query,Modules).

%% 
% @implements 'db/itripledb'
%
tripledb_cache_add(Predicate,Query,Module) :-
  itripledb_cache_add(Predicate,Query,Module).

%% 
% @implements 'db/itripledb'
%
tripledb_cache_invalidate(Predicate) :-
  itripledb_cache_invalidate(Predicate).
