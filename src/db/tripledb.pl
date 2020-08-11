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

:- use_module(library('utility/url'),    [ url_resolve/2, url_unresolve/2 ]).
:- use_module(library('db/scope'),       [ universal_scope/1 ]).
:- use_module(library('comm/notify'),    [ notify/1 ]).
:- use_module(library('model/XSD'),      [ xsd_data_basetype/2 ]).

% define some settings
:- setting(path, atom, 'db/mongo/tripledb/plugin',
		'Path to the module where the triple DB is implemented.').
:- setting(drop, boolean, false,
		'Toggle to configure whether the DB should initially by erased.').

%%
% Get path to module that implements the tripledb
% and import prefixed predicates from the module.
% TODO: rather do re-export? not sure if this kind of wrapping
%       adds much overhead.
:- setting(tripledb:path,Module),
   use_module(library(Module),
        [ tripledb_init/0             as itripledb_init,
          tripledb_import/1           as itripledb_import,
          tripledb_export/1           as itripledb_export,
          tripledb_drop/0             as itripledb_drop,
          tripledb_tell/5             as itripledb_tell,
          tripledb_bulk_tell/3        as itripledb_bulk_tell,
          tripledb_ask/6              as itripledb_ask,
          tripledb_forget/5           as itripledb_forget,
          tripledb_cache_get/3        as itripledb_cache_get,
          tripledb_cache_add/3        as itripledb_cache_add,
          tripledb_cache_invalidate/1 as itripledb_cache_invalidate
        ]).
% drop the triple DB initially if requested by the user.
:- setting(tripledb:drop,true) -> itripledb_drop ; true.

%%
:- dynamic default_graph/1.
default_graph(user).

%%
% Set the name of the graph where triples are asserted and retrieved
% if no other graph was specified.
%
set_default_graph(Graph) :-
  retractall(default_graph(_)),
  assertz(default_graph(Graph)).

set_graph_option(Options,Options) :-
  option(graph(_),Options),!.

set_graph_option(Options,Merged) :-
  default_graph(DG),
  merge_options([graph(DG)],Options,Merged).

%% tripledb_load(+URL) is det.
%
% Same as tripledb_load/2 with empty Options list.
%
% @param URL URL of a RDF file.
% 
tripledb_load(URL) :-
  tripledb_load(URL,[]).

%% tripledb_load(+URL,+Options) is semidet.
%
% Same as tripledb_load/3 with universal scope,
% and graph name argument as given in the options list
% or "user" if none is given.
%
% @param URL URL of a RDF file.
% @param Options List of options.
%
tripledb_load(URL,Options) :-
  is_list(Options),!,
  % register namespace
  ( member(namespace(NS),Options) ->
    ( atom_concat(URL,'#',Prefix),
      rdf_register_ns(NS,Prefix,[keep(true)]));
    ( true )
  ),
  ( member(namespace(NS,Prefix),Options) ->
    ( rdf_register_ns(NS,Prefix,[keep(true)]));
    ( true )
  ),
  % get graph name
  ( member(graph(Graph),Options) ->
    ( true );
    ( default_graph(Graph) )
  ),
  % get fact scope
  universal_scope(Scope),
  tripledb_load(URL,Scope,Graph).

%% tripledb_load(+URL,+Scope,+Graph) is semidet.
%
% Load RDF data from file, and assert it into
% the triple DB using the scope provided, and
% into the graph with the name provided.
%
% @param URL URL of a RDF file.
% @param Scope The subject of a triple.
% @param Graph The graph name.
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
  %%
  once(( url_unresolve(URL,Unresolved) ; Unresolved=URL )),
  universal_scope(US),
  tripledb_tell(Unresolved,rdf:type,string(owl:'Ontology'),US,[graph(Graph)]),
  % load data into triple DB
  print_message(informational,tripledb(load(URL_resolved))),
  tripledb_load3t(Unresolved,Triples,Scope,Graph),
  % notify about asserted individuals
  forall(
    member(rdf(X,RDF_Type,OWL_NamedIndividual), Triples),
    ignore(notify(individual(X)))
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
tripledb_load3t(IRI,Triples,Scope,Graph) :-
  % debug how long loading takes
  length(Triples,NumTriples),
  get_time(Time0),
  tripledb_load3(IRI,Triples,Scope,Graph),
  get_time(Time1),
  PerSec is NumTriples/(Time1-Time0),
  print_message(informational, tripledb(loaded(ntriples(NumTriples),persecond(PerSec)))).

%%
tripledb_load3(IRI,Triples,Scope,Graph) :-
  findall(Converted, (
    member(Triple0,Triples),
    convert_rdf_(IRI,Triple0,Converted)
  ), ConvertedTriples),
  tripledb_bulk_tell(ConvertedTriples,Scope,[graph(Graph)]).

%%
convert_rdf_(IRI,rdf(S,P,O),rdf(S1,P,O2)) :-
  convert_blank_node_(IRI,S,S1),
  convert_blank_node_(IRI,O,O1),
  convert_rdf_value_(O1,O2),
  !.

%%
convert_blank_node_(IRI,Blank,Converted) :-
	% avoid name clashes between blanks loaded from different ontologies
	% by prepending the ontology prefix. 
	( ( atom(Blank), atom_concat('_:',_,Blank) )
	-> atomic_list_concat([IRI,'#',Blank],'',Converted)
	;  Converted=Blank
	).

%%
convert_rdf_value_(literal(type(Type,V_atom)),O_typed) :-
  xsd_data_basetype(Type,TypeKey),
  ( TypeKey=integer -> atom_number(V_atom,V_typed)
  ; TypeKey=double  -> atom_number(V_atom,V_typed)
  ; V_typed=V_atom
  ),
  O_typed=..[TypeKey,V_typed].
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
tripledb_drop :-
  itripledb_drop.

%% 
% @implements 'db/itripledb'
%
tripledb_tell(S,P,O,Scope,Options) :-
%  ( option(functional,Options)
%  -> tripledb_stop(S,P,Scope,Options)
%  ;  true
%  ),
  set_graph_option(Options,Options0),
  itripledb_tell(S,P,O,Scope,Options0).

%% tripledb_tell(?S,?P,?O,+Scope) is semidet.
%
% Same as tripledb_tell/5 but uses universal 
% scope, and an empty options list.
%
% @param S The subject of a triple.
% @param P The predicate of a triple.
% @param O The object or data value of a triple.
% @param Scope The scope of the new triple.
%
tripledb_tell(S,P,O,Scope) :-
  tripledb_tell(S,P,O,Scope,[]).

%% tripledb_tell(?S,?P,?O) is semidet.
%
% Same as tripledb_tell/5 but uses universal 
% scope, and an empty options list.
%
% @param S The subject of a triple.
% @param P The predicate of a triple.
% @param O The object or data value of a triple.
%
tripledb_tell(S,P,O) :-
  universal_scope(Scope),
  tripledb_tell(S,P,O,Scope,[]).

%% 
% @implements 'db/itripledb'
%
tripledb_bulk_tell(Facts,Scope,Options) :-
  itripledb_bulk_tell(Facts,Scope,Options).

%%
tripledb_stop(S,P,Scope,Options) :-
	% TODO implement
	true.

%% 
% @implements 'db/itripledb'
%
tripledb_forget(S,P,O,Scope,Options) :-
  set_graph_option(Options,Options0),
  itripledb_forget(S,P,O,Scope,Options0).

%% tripledb_forget(?S,?P,?O,+Scope) is semidet.
%
% Same as tripledb_forget/5 but uses an empty options list.
%
% @param S The subject of a triple.
% @param P The predicate of a triple.
% @param O The object or data value of a triple.
% @param Scope The scope of considered triples.
%
tripledb_forget(S,P,O,Scope) :-
  tripledb_forget(S,P,O,Scope,[]).

%% tripledb_forget(?S,?P,?O) is semidet.
%
% Same as tripledb_forget/5 but uses wildcard 
% scope, and an empty options list.
%
% @param S The subject of a triple.
% @param P The predicate of a triple.
% @param O The object or data value of a triple.
%
tripledb_forget(S,P,O) :-
  wildcard_scope(QScope),
  tripledb_forget(S,P,O,QScope,[]).

%% 
% @implements 'db/itripledb'
%
tripledb_ask(S,P,O,QScope,FScope,Options) :-
  set_graph_option(Options,Options0),
  itripledb_ask(S,P,O,QScope,FScope,Options0).

%% tripledb_ask(?S,?P,?O,+QScope,-FScope) is semidet.
%
% Same as tripledb_ask/6 but uses an empty options list.
%
% @param S The subject of a triple.
% @param P The predicate of a triple.
% @param O The object or data value of a triple.
% @param QScope The scope of requested triples.
% @param FScope The scope of the retrieved triple.
%
tripledb_ask(S,P,O,QScope,FScope) :-
  tripledb_ask(S,P,O,QScope,FScope,[]).

%% tripledb_ask(?S,?P,?O) is semidet.
%
% Same as tripledb_ask/6 but uses wildcard 
% scope, and an empty options list.
%
% @param S The subject of a triple.
% @param P The predicate of a triple.
% @param O The object or data value of a triple.
%
tripledb_ask(S,P,O) :-
  wildcard_scope(QScope),
  tripledb_ask(S,P,O,QScope,_,[]).

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
