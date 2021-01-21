:- module(lang_export,
    [ tripledb_load/1,
      tripledb_load/2,
      tripledb_load/3,
      remember/1,
      memorize/1
    ]).
/** <module> Interface for dumping knowledge and restoring it.

@author Daniel Beßler
@license BSD
*/

:- use_module(library(rdf),              [ load_rdf/3 ]).
:- use_module(library('semweb/rdf_db'),  [ rdf_equal/2, rdf_register_ns/3 ]).
:- use_module(library('http/http_open'), [ http_open/3 ]).

:- use_module(library('utility/url'),
		[ url_resolve/2 ]).
:- use_module(library('model/XSD'),
		[ xsd_data_basetype/2 ]).
:- use_module(library('db/scope'),
		[ universal_scope/1,
		  wildcard_scope/1
		]).
:- use_module(library('db/subgraph')).

:- multifile remember_hook/1.
:- multifile memorize_hook/1.

:- dynamic kb_collection_name/1.

%%
% Needs to be called for each DB collection to be
% taken into account during import/export.
%
kb_collection_init(Name) :-
	assert(kb_collection_name(Name)).

%% remember(+Directory) is det.
%
% Restore memory previously stored into given directory.
%
% @param Directory filesystem path
%
remember(Directory) :-
	mng_import(Directory),
	forall(remember_hook(Directory), true).

%%
mng_import(Dir) :-
	mng_db_name(DB),
	forall(
		kb_collection_name(Collection),
		(	path_concat(Dir, Collection, Dir0),
			mng_restore(DB, Dir0)
		)
	).

%% memorize(+Directory) is det.
%
% Store knowledge into given directory.
%
% @param Directory filesystem path
%
memorize(Directory) :-
	mng_export(Directory),
	forall(memorize_hook(Directory), true).

%%
mng_export(Dir) :-
	mng_db_name(DB),
	forall(
		mng_collection_name(Collection),
		(	path_concat(Dir, Collection, Dir0),
			mng_dump_collection(DB, Collection, Dir0)
		)
	).

     /*******************************
     *    LOADING ONTOLOGIES        *
     *******************************/

%% tripledb_load(+URL) is det.
%
% Same as tripledb_load/2 with empty Options list.
%
% @param URL URL of a RDF file.
% 
tripledb_load(URL) :-
	tripledb_load(URL, []).

%% tripledb_load(+URL,+Options) is semidet.
%
% Same as tripledb_load/3 with universal scope,
% and graph name argument as given in the options list
% or "user" if none is given.
%
% @param URL URL of a RDF file.
% @param Options List of options.
%
tripledb_load(URL, Options) :-
	is_list(Options),
	!,
	% register namespace
	(	member(namespace(NS), Options)
	->	(	atom_concat(URL, '#', Prefix),
			rdf_register_ns(NS, Prefix, [keep(true)])
		)
	;	true
	),
	(	member(namespace(NS,Prefix), Options)
	->	rdf_register_ns(NS, Prefix, [keep(true)])
	;	true
	),
	% get graph name
	(	member(graph(Graph), Options)
	->	true
	;	Graph=_
	),
	% get fact scope
	universal_scope(Scope),
	tripledb_load(URL, Scope, Graph).

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
tripledb_load(URL, Scope, SubGraph) :-
	(	url_resolve(URL,Resolved)
	->	true
	;	Resolved=URL 
	),
	ontology_graph(Resolved,OntoGraph),
	%% setup graph structure
	(	SubGraph == common
	->	true
	;	(	add_subgraph(OntoGraph,common),
			add_subgraph(user,OntoGraph)
		)
	),
	(	ground(SubGraph)
	->	(	add_subgraph(SubGraph,OntoGraph),
			add_subgraph(user,SubGraph)
		)
	;	true
	),
	!,
	%%
	(	setting(mng_client:read_only, true)
	->	true
	;	tripledb_load0(Resolved, Scope, OntoGraph, SubGraph)
	).

tripledb_load0(Resolved,_,OntoGraph,_) :-
	% test whether the ontology is already loaded
	get_ontology_version(OntoGraph,Version),
	file_version(Resolved,Version),
	% no triples to load if versions unify
	% but we must make sure the graph is a sub-graph
	% of all imported ontologies.
	wildcard_scope(QScope),
	forall(
		once(mng_ask(
			triple(_, owl:'imports', string(Imported)),
			QScope, _FScope,
			[graph(=(OntoGraph))]
		)),
		(	ontology_graph(Imported,ImportedGraph),
			add_subgraph(OntoGraph,ImportedGraph)
		)
	),
	!.

tripledb_load0(Resolved,Scope,OntoGraph,SubGraph) :-
	rdf_equal(owl:'imports',OWL_Imports),
	rdf_equal(owl:'Ontology',OWL_Ontology),
	rdf_equal(rdf:'type',RDF_Type),
	% erase old triples
	triple_graph_drop(OntoGraph),
	%
	load_rdf_(Resolved, Triples),
	% get ontology IRI
	(	member(rdf(Unresolved,RDF_Type,OWL_Ontology), Triples)
	->	true
	;	(	log_error(type_error(ontology,Resolved)),
			fail
		)
	),
	% first, load RDF data of imported ontologies
	forall(
		member(rdf(Unresolved,OWL_Imports,I), Triples),
		(	ontology_graph(I,ImportedGraph),
			add_subgraph(OntoGraph,ImportedGraph),
			tripledb_load(I,Scope,SubGraph)
		)
	),
	% assert a version string
	file_version(Resolved, Version),
	set_ontology_version(Unresolved, Version, OntoGraph),
	% load data into triple DB
	tripledb_load1(Unresolved,Triples,Scope,OntoGraph),
	!.

%%
tripledb_load1(IRI, Triples, Scope, Graph) :-
	% debug how long loading takes
	get_time(Time0),
	maplist(convert_rdf_(IRI), Triples, Terms),
	% TODO: better use bulk insert interface
	lang_query:tell(Terms, [[graph(Graph)],Scope]),
	get_time(Time1),
	% debug
	length(Triples,NumTriples),
	PerSec is NumTriples/(Time1-Time0),
	log_debug(tripledb(loaded(
		ntriples(NumTriples),persecond(PerSec)))).

% wrapper around load_rdf/3
load_rdf_(URL,Triples) :-
	sub_string(URL,0,4,_,'http'),
	!,
	http_open(URL, RDF_Stream, []),
	load_rdf_1(RDF_Stream, Triples),
	close(RDF_Stream).

load_rdf_(URL, Triples) :-
	load_rdf_1(URL, Triples).

load_rdf_1(URL, Triples) :-
	% load rdf data. *Triples* is a ist of
	% `rdf(Subject, Predicate, Object)` terms.
	load_rdf(URL, Triples, [blank_nodes(noshare)]).

%%
convert_rdf_(IRI, rdf(S,P,O), triple(S1,P,O2)) :-
	convert_blank_node_(IRI,S,S1),
	convert_blank_node_(IRI,O,O1),
	convert_rdf_value_(O1,O2).

% avoid name clashes between blanks loaded from different ontologies
% by prepending the ontology prefix. 
convert_blank_node_(IRI,Blank,Converted) :-
	(	is_blank(Blank)
	->	atomic_list_concat([IRI,'#',Blank],'',Converted)
	;	Converted=Blank
	).

%%
is_blank(Blank) :-
	atom(Blank),
	atom_concat('_:',_,Blank).

% convert typed literals to expected format
convert_rdf_value_(literal(type(Type,V_atom)), O_typed) :-
	xsd_data_basetype(Type, TypeKey),
	(	TypeKey=integer -> atom_number(V_atom,V_typed)
	;	TypeKey=double  -> atom_number(V_atom,V_typed)
	;	V_typed=V_atom
	),
	O_typed=..[TypeKey,V_typed],
	!.

convert_rdf_value_(literal(O), string(O)) :- !.
convert_rdf_value_(        O,  string(O)) :- !.

% each ontology is stored in a separate graph named
% according to the ontology
ontology_graph(URL,Name) :-
	file_base_name(URL,FileName),
	file_name_extension(Name,_,FileName),
	!.

     /*******************************
     *          VERSIONING          *
     *******************************/

%% The version/last modification time of a loaded ontology
get_ontology_version(OntoGraph, Version) :-
	wildcard_scope(QScope),
	% find value of property "tripledbVersionString" in ontology graph
	once(mng_ask(
		triple(_, tripledbVersionString, string(Version)),
		QScope, _FScope,
		[graph(=(OntoGraph))]
	)).

%% Write version string into DB
set_ontology_version(URL, Version, OntoGraph) :-
	universal_scope(Scope),
	once(mng_tell(
		triple(URL, tripledbVersionString, string(Version)),
		Scope,
		[graph(=(OntoGraph))]
	)).

%% get modification time of local file.
%% this is to cause re-loading the file in case of it has changed locally.
file_version(URL, Version) :-
	exists_file(URL),
	!,
	set_time_file(URL, [modified(ModStamp)],[]),
	atom_number(Version, ModStamp).

%% try to extract version from URI
file_version(URL, Version) :-
	atomic_list_concat(XL,'/',URL),
	% TODO: some ontologies use e.g. ".../2005/07/xx.owl"
	reverse(XL, [_,Version|_]),
	is_version_string(Version),
	!.

%% remote URL -> reload only one update per day
file_version(_URL, Version) :-
	date(VersionTerm),
	term_to_atom(VersionTerm, Version).

%% version string validation
is_version_string(Atom) :-
	atom_codes(Atom,Codes),
	phrase(version_matcher,Codes),
	!.

version_matcher --> "v", version_matcher.
version_matcher --> digits(_), ".", digits(_), ".", digits(_).
version_matcher --> digits(_), ".", digits(_).

     /*******************************
     *          UNIT TESTS          *
     *******************************/

:- begin_tests('lang_export').

get_path(Path):-
  working_directory(X,X), string_concat(X, "test_lang_export", Path).

test('stores knowledge in test_lang_export directory and restores the same', 
  [ blocked('Not possible to set a different database in runtime at the moment'),
    setup(get_path(Path)),
    cleanup(shell('cd $(rospack find knowrob); rm -rf test_lang_export'))
  ]) :-
  assert_true(memorize(Path)),
  assert_true(tripledb_whipe),
  assert_true(remember(Path)).

:- end_tests('lang_export').
