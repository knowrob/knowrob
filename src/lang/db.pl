:- module(lang_db,
    [ load_owl/1,
      load_owl/2,
      load_owl/3,
      load_json_rdf/1,
      remember/1,
      memorize/1,
      drop_graph/1,
      get_unique_name(r,-),
      is_unique_name(r),
      setup_collection/2
    ]).
/** <module> Interface for dumping knowledge and restoring it.

@author Daniel Beßler
@license BSD
*/

:- use_module(library(rdf),
		[ load_rdf/3 ]).
:- use_module(library('semweb/rdf_db'), 
		[ rdf_equal/2, rdf_register_ns/3 ]).
:- use_module(library('http/http_open'),
		[ http_open/3 ]).
:- use_module(library('utility/url'),
		[ url_resolve/2 ]).
:- use_module(library('model/XSD'),
		[ xsd_data_basetype/2 ]).
:- use_module(library('db/mongo/client')).
:- use_module('scope',
		[ universal_scope/1 ]).
:- use_module('subgraph').

% define some settings
:- setting(drop_graphs, list, [user],
		'List of named graphs that should initially by erased.').

%%
:- multifile collection_name/1.

%%
setup_collection(Name, Indices) :-
	assertz(collection_data_(Name, Indices)),
	create_indices(Name, Indices).

%% remember(+Directory) is det.
%
% Restore memory previously stored into given directory.
%
% @param Directory filesystem path
%
remember(Directory) :-
	mng_import(Directory).

%%
mng_import(Dir) :-
	forall(
		(	collection_name(Name),
			mng_get_db(DB, Collection, Name)
		),
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
	mng_export(Directory).

%%
mng_export(Dir) :-
	forall(
		(	collection_name(Name),
			mng_get_db(DB, Collection, Name)
		),
		(	path_concat(Dir, Collection, Dir0),
			mng_dump_collection(DB, Collection, Dir0)
		)
	).

%% drop_graph(+Name) is det.
%
% Deletes all triples asserted into given named graph.
%
% @Name the graph name.
%
drop_graph(Name) :-
	mng_get_db(DB, Coll, 'triples'),
	mng_remove(DB, Coll, [
		[graph, string(Name)]
	]).

%% is_unique_name(+Name) is semidet.
%
% True is Name is not the subject of any known fact.
%
is_unique_name(Name) :-
	mng_get_db(DB, Coll, 'triples'),
	\+ mng_find(DB, Coll, [['s',string(Name)]], _).

%% get_unique_name(+Prefix, -Name) is semidet.
%
% Generates a unique name with given prefix.
%
get_unique_name(Prefix, Name) :-
	% generate 8 random alphabetic characters
	randseq(8, 25, Seq_random),
	maplist(plus(65), Seq_random, Alpha_random),
	atom_codes(Sub, Alpha_random),
	% TODO: what IRI prefix? Currently we re-use the one of the type.
	%        but that seems not optimal. Probably best to
	%        have this in query context, and some meaningful default.
	atomic_list_concat([Prefix,'_',Sub], IRI),
	% check if there is no triple with this identifier as subject or object yet
	(	is_unique_name(IRI)
	->	Name=IRI
	;	unique_name(Prefix,Name)
	).

%%
% Drop graphs on startup if requested through settings.
% This is usually done to start with an empty "user" graph
% when KnowRob is started.
%
auto_drop_graphs :-
	\+ setting(mng_client:read_only, true),
	setting(lang_db:drop_graphs, L),
	forall(member(X,L), drop_graph(X)).

:- ignore(auto_drop_graphs).

%% load_owl(+URL) is det.
%
% Same as load_owl/2 with empty Options list.
%
% @param URL URL of a RDF file.
% 
load_owl(URL) :-
	load_owl(URL, []).

%% load_owl(+URL,+Options) is semidet.
%
% Same as load_owl/3 with universal scope,
% and graph name argument as given in the options list
% or "user" if none is given.
%
% @param URL URL of a RDF file.
% @param Options List of options.
%
load_owl(URL, Options) :-
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
	(	member(graph(Graph), Options) -> true
	;	Graph=_
	),
	% get fact scope
	universal_scope(Scope),
	load_owl(URL, Scope, Graph).

%% load_owl(+URL,+Scope,+Graph) is semidet.
%
% Load RDF data from file, and assert it into
% the triple DB using the scope provided, and
% into the graph with the name provided.
%
% @param URL URL of a RDF file.
% @param Scope The subject of a triple.
% @param Graph The graph name.
%
load_owl(URL, Scope, SubGraph) :-
	(	url_resolve(URL,Resolved) -> true
	;	Resolved=URL 
	),
	(	ground(SubGraph) -> OntoGraph=SubGraph
	;	ontology_graph(Resolved,OntoGraph)
	),
	%% setup graph structure
	(	add_subgraph(OntoGraph,common),
		add_subgraph(user,OntoGraph)
	),
	!,
	%%
	(	setting(mng_client:read_only, true) -> true
	;	load_owl0(Resolved, Scope, OntoGraph, SubGraph)
	).

load_owl0(Resolved, _, OntoGraph, _SubGraph) :-
	rdf_equal(owl:'imports',OWL_Imports),
	% test whether the ontology is already loaded
	get_ontology_version(OntoGraph,Version),
	file_version(Resolved,Version),
	% no triples to load if versions unify
	% but we must make sure the graph is a sub-graph
	% of all imported ontologies.
	mng_get_db(DB, Coll, 'triples'),
	forall(
		mng_find(DB, Coll, [
			['p',     string(OWL_Imports)],
			['graph', string(OntoGraph)]
		], Doc),
		(	mng_get_dict('o', Doc, string(Imported)),
			ontology_graph(Imported,ImportedGraph),
			add_subgraph(OntoGraph,ImportedGraph)
		)
	),
	!,
	log_info(db(ontology_detected(OntoGraph,Version))).

load_owl0(Resolved,Scope,OntoGraph,SubGraph) :-
	rdf_equal(owl:'imports',OWL_Imports),
	rdf_equal(owl:'Ontology',OWL_Ontology),
	rdf_equal(rdf:'type',RDF_Type),
	% erase old triples
	(	ground(SubGraph) -> true
	;	drop_graph(OntoGraph)
	),
	%
	load_rdf_(Resolved, Triples),
	% get ontology IRI
	(	member(rdf(Unresolved,RDF_Type,OWL_Ontology), Triples) -> true
	;	log_error_and_fail(type_error(ontology,Resolved))
	),
	% first, load RDF data of imported ontologies
	forall(
		member(rdf(Unresolved,OWL_Imports,I), Triples),
		(	ontology_graph(I,ImportedGraph),
			add_subgraph(OntoGraph,ImportedGraph),
			load_owl(I,Scope,SubGraph)
		)
	),
	% assert a version string
	file_version(Resolved, Version),
	set_ontology_version(Unresolved, Version, OntoGraph),
	% load data into triple DB
	load_owl1(Unresolved,Triples,Scope,OntoGraph),
	!,
	log_info(db(ontology_loaded(OntoGraph,Version))).

%%
load_owl1(IRI, Triples, Scope, Graph) :-
	% debug how long loading takes
	get_time(Time0),
	maplist(convert_rdf_(IRI), Triples, Terms),
	lang_query:tell(Terms, Scope, [graph(Graph)]),
	get_time(Time1),
	% debug
	length(Triples,NumTriples),
	PerSec is NumTriples/(Time1-Time0),
	log_debug(tripledb(loaded(
		ntriples(NumTriples),persecond(PerSec)))).

%% reads json data and asserts into mongodb
%
% @param FilePath - Path to the json file
%
load_json_rdf(FilePath) :-
	open(FilePath,read,Stream),
	read_data(Stream,_Triples),
	close(Stream).

read_data(Stream,[]):-
	at_end_of_stream(Stream),
	!.

read_data(Stream,[TriplesDict | Rest]):-
	json:json_read_dict(Stream, TriplesDict),
	assert_triple_data(TriplesDict),
	read_data(Stream,Rest).

assert_triple_data(Triples) :-
	term_to_atom(Triples.get(s), S),
	term_to_atom(Triples.get(p), P),
	term_to_atom(Triples.get(o), O),
	% TODO: also support to import scope information
	% TODO: it would be faster to call tell only once with
	% array of triples
	lang_query:tell(triple(S, P, O)).

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
	mng_get_db(DB, Coll, 'triples'),
	once(mng_find(DB, Coll, [
		['p',     string(tripledbVersionString)],
		['graph', string(OntoGraph)]
	], Doc)),
	mng_get_dict(o, Doc, string(Version)).

%% Write version string into DB
set_ontology_version(URL, Version, OntoGraph) :-
	mng_get_db(DB, Coll, 'triples'),
	mng_store(DB, Coll, [
		['s',     string(URL)],
		['p',     string(tripledbVersionString)],
		['o',     string(Version)],
		['graph', string(OntoGraph)]
	]).

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
     *        SEARCH INDEX          *
     *******************************/

%%
create_indices :-
	forall(
		collection_data_(Name, Indices),
		create_indices(Name, Indices)
	).

%% Create indices for fast annotation retrieval.
create_indices(_Name, _Indices) :-
	setting(mng_client:read_only, true),
	!.
create_indices(Name, Indices) :-
	mng_get_db(DB, Coll, Name),
	forall(
		member(Index, Indices),
		mng_index_create(DB, Coll, Index)
	).

     /*******************************
     *    SPECIAL COLLECTIONS       *
     *******************************/

%%
% True if "one" collection has a document.
%
has_one_db :-
	mng_one_db(DB, Collection),
	mng_find(DB, Collection, [], _),
	!.

%%
% Ensure that "one" collection has one document.
%
initialize_one_db :-
	has_one_db, !.

initialize_one_db :-
	mng_one_db(DB, Collection),
	mng_store(DB, Collection, [
		['v_scope', [
			['time', [
				['since',double(0)],
				['until',double('Infinity')]
			]]
		]]
	]).

% make sure collection "one" has a document
:- initialize_one_db.

