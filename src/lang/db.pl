:- module(lang_db,
    [ load_owl/1,
      load_owl/2,
      load_owl/3,
      load_json_rdf/1,
      remember/1,
      memorize/1,
      watch(t,t,-),
      watch_event(+,+),
      unwatch(+),
      drop_graph/1,
      get_unique_name(r,-),
      is_unique_name(r),
      setup_collection/2
    ]).
/** <module> Database predicates.

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
:- setting(mng_client:collection_names, list, [triples, tf, annotations, inferred],
		'List of collections that will be imported/exported with remember/memorize.').

%%
annotation_property('http://www.w3.org/2000/01/rdf-schema#comment').
annotation_property('http://www.w3.org/2000/01/rdf-schema#seeAlso').
annotation_property('http://www.w3.org/2000/01/rdf-schema#label').
annotation_property('http://www.w3.org/2002/07/owl#versionInfo').

%%
% Assert the collection names to be used by remember/memorize
%
auto_collection_names :-
	setting(mng_client:collection_names, L),
	forall(member(X,L), assertz(collection_name(X))).

:- ignore(auto_collection_names).

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
% @param Name the graph name.
%
drop_graph(Name) :-
	mng_get_db(DB, Coll, 'triples'),
	mng_remove(DB, Coll, [
		[graph, string(Name)]
	]).

%% is_unique_name(+Name) is semidet.
%
% True if Name is not the subject of any known fact.
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

%% setup_collection(+Name, +Indices) is det.
%
% Configure the indices of a named collection.
%
setup_collection(Name, Indices) :-
	assertz(collection_data_(Name, Indices)),
	create_indices(Name, Indices).

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
	(	option(namespace(NS), Options)
	->	(	atom_concat(URL, '#', Prefix),
			rdf_register_ns(NS, Prefix, [keep(true)])
		)
	;	true
	),
	(	option(namespace(NS,Prefix), Options)
	->	rdf_register_ns(NS, Prefix, [keep(true)])
	;	true
	),
	% get parent graph name, fall back to "common"
	option(parent_graph(ParentGraph), Options, common),
	% get fact scope
	universal_scope(Scope),
	load_owl(URL, Scope, ParentGraph).

%% load_owl(+URL,+Scope,+ParentGraph) is semidet.
%
% Load RDF data from URL, and assert it into
% the triple DB using the scope provided
% and into a graph named according to the ontology.
% The named graph is added as child graph of ParentGraph.
%
% @param URL URL of a RDF file.
% @param Scope The subject of a triple.
% @param ParentGraph The parent graph name.
%
load_owl(URL, Scope, ParentGraph) :-
	(	url_resolve(URL,Resolved) -> true
	;	Resolved=URL 
	),
	ontology_graph(Resolved,OntoGraph),
	%% setup graph structure
	(	add_subgraph(OntoGraph,ParentGraph),
		add_subgraph(user,OntoGraph)
	),
	!,
	%%
	(	setting(mng_client:read_only, true) -> true
	;	load_owl0(Resolved, Scope, OntoGraph, ParentGraph)
	).

load_owl0(Resolved, _, OntoGraph, _ParentGraph) :-
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

load_owl0(Resolved, Scope, OntoGraph, ParentGraph) :-
	rdf_equal(owl:'imports',OWL_Imports),
	rdf_equal(owl:'Ontology',OWL_Ontology),
	rdf_equal(rdf:'type',RDF_Type),
	% erase old triples
	drop_graph(OntoGraph),
	%
	load_rdf_(Resolved, Triples),
	% get ontology IRI
	(	member(rdf(Unresolved,RDF_Type,OWL_Ontology), Triples) -> true
	;	log_error_and_fail(type_error(ontology,Resolved))
	),
	% first, load RDF data of imported ontologies
	forall(
		member(rdf(Unresolved,OWL_Imports,I), Triples),
		(	ontology_graph(I, ImportedGraph),
			add_subgraph(OntoGraph, ImportedGraph),
			load_owl(I, Scope, ParentGraph)
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
	maplist(convert_rdf_(IRI), Triples, Terms),
	% NOTE: annotations are stored in a separate collection.
	%       the reason is that we create a search index over the value
	%       of a triple, and that mongo cannot generate such an index
	%       over values with special characters.
	exclude(is_annotation_triple, Terms, TripleTerms),
	include(is_annotation_triple, Terms, AnnotationTriples),
	maplist([triple(S,P,O),annotation(S,P,O)]>>true,
		AnnotationTriples, AnnotationTerms),
	% FIXME: BUG: o* for subClassOf only includes direct super class
	%             when loading a list of triples at once.
	%             this does not seem to occur when loading each triple
	%             individually.
	%kb_project(TripleTerms, Scope, [graph(Graph)]),
	%kb_project(AnnotationTerms, Scope, [graph(Graph)]),
	forall(
		(	member(Term, TripleTerms)
		;	member(Term, AnnotationTerms)
		),
		kb_project(Term, Scope, [graph(Graph)])
	).

%% load_json_rdf(FilePath) is semidet.
%
% Load JSON-encoded triple data into the knowledge base.
% Each triple document in the JSON file must have the keys
% "s","p","o" for the subject, property, and value of the triple.
% In addition a scope document can be provided optionally.
% If this is not the case, it is assumed that the facts universally hold.
%
% @param FilePath - Path to the json file
%
load_json_rdf(FilePath) :-
	open(FilePath,read,Stream),
	read_data(Stream,_Triples),
	close(Stream).

read_data(Stream,[]):-
	at_end_of_stream(Stream).

read_data(Stream,[TriplesDict | Rest]):-
	json:json_read_dict(Stream, TriplesDict),
	assert_triple_data(TriplesDict),
	read_data(Stream,Rest).

assert_triple_data(Triples) :-
	is_dict(Triples),!,
	get_dict(s, Triples, S),
	get_dict(p, Triples, P),
	get_dict(o, Triples, O),
	triple_json_scope(Triples,Scope),
	triple_json_object(O,O_value),
	atom_string(S_atom, S),
	atom_string(P_atom, P),
	% TODO: it would be faster to call assert only once with
	% array of triples
	kb_call(assert(triple(S_atom, P_atom, O_value)), Scope, _).

assert_triple_data(TriplesList) :-
	%handle case when given triples are list
	is_list(TriplesList),!,
	forall(member(X,TriplesList), assert_triple_data(X)).

%% triple_json_object(+Dict,-O) is semidet.
triple_json_object(Dict,O) :-
	% if the argument is a dictionary
	is_dict(Dict),!,
	% FIXME: below does not look right
	get_dict('$numberDecimal', Dict, Json_Object),
	(	atom(Json_Object)   -> atom_number(Json_Object,O)
	;	string(Json_Object) -> number_string(O,Json_Object)
	;	O is Json_Object
	).

triple_json_object(O,O).

triple_json_scope(Triples,Scope) :-
    % check if 'since' and 'until' are part of triple, if not then create universal scope(0 to Inf)
    get_dict(since, Triples, Since),
	get_dict(until, Triples, Until),!,
	% check if given 'Since' and 'Until' are numbers if they are not
	% then convert them into numbers first and then use them into scope
	(	number(Since) -> Since_number = Since
	;	atom_number(Since, Since_number)
	),
	(	number(Until) -> Until_number = Until
	;	atom_number(Until, Until_number)
	),
	time_scope(Since_number, Until_number, Scope).

triple_json_scope(_Triples,Scope) :-
	% create universal scope when either of 'since' or 'until' are not provided in triple
	universal_scope(Scope).

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

%%
is_annotation_triple(triple(_,P,_)) :-
	once(annotation_property(P)).

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
     *          DB EVENTS           *
     *******************************/

%% watcher(+WatcherID, +Goal)
%
% Stores information about active watch processes.
%
:- dynamic watcher/2.

%% watch(+Goal, +Callback, -WatcherID) is semidet.
%
% Start watching possible instantiations of variables in Goal.
% Goal must be a KnowRob language term.
% Callback is called whenever the set of possible instantiations changes,
% and it is provided with information about the change.
% The change information is encoded in a term that is appended to
% already existing arguments of Callback, if any.
%
% Note that callback is currently not called when documents are removed!
% This might change in the future.
%
% Also note that currently Goal must be a term triple/3.
% The scope of triples is ignored.
% No other predicates are supported yet.
% This might also change in the future.
%
% @param Goal a KnowRob language term with free variables.
% @param Callback a predicate called for each change event.
% @param WatcherID a unique identifier of the watching operation.
%
watch(Goal, Callback, WatcherID) :-
	% create match filter based on Goal
	% NOTE: it is not possible to $match remove events like this
	%       because the document values are _not_ included in the event :/
	%       this means additional bookkeeping is needed to be able to handle
	%       remove events ...
	watch_filter(Goal, DB, Coll, WatchFilter),
	% start receiving events about documents that match the filter
	mng_watch(DB, Coll,
		watch_event,
		[pipeline, array([
			['$match', WatchFilter]
		])],
		WatcherID),
	% FIXME: there is a risk watch_event is called
	%        before this fact is asserted
	assertz(watcher(WatcherID, Callback)).

%% unwatch(+WatcherID) is semidet.
%
% Stop a previously started watch operation.
% This will fail if WatcherID does not belong to
% a active watch operation.
%
% @param WatcherID a unique identifier of the watching operation.
%
unwatch(WatcherID) :-
	once(watcher(WatcherID, _)),
	mng_unwatch(WatcherID),
	retractall(watcher(WatcherID, _)).

%% watch_event(+WatcherID, +Event) is semidet.
%
% This predicate is called by the mongo client when
% changes are received in a watch operation.
% Event is a change stream response document.
%
% @param WatcherID a unique identifier of the watching operation.
% @param Event a change stream output document.
%
% @see https://docs.mongodb.com/manual/reference/change-events/#change-stream-output
%
watch_event(WatcherID, Event) :-
	once(watcher(WatcherID, CallbackGoal)),
	dict_pairs(Dict, _, Event),
	mng_get_dict(operationType, Dict, string(OpType)),
	memberchk(OpType, [insert, update]),
	% read the document from "fullDocument" field
	mng_get_dict(fullDocument,  Dict, FullDoc),
	mng_get_dict('_id', FullDoc, id(DocID)),
	% parse triple
	% TODO: support other terms here
	mng_get_dict(s, FullDoc, string(S)),
	mng_get_dict(p, FullDoc, string(P)),
	mng_get_dict(o, FullDoc, TypedValue),
	mng_strip_type(TypedValue, _, Value),
	% finally call the watcher
	UpdateTerm = event(OpType, [
		id(DocID),
		new_value(triple(S,P,Value))
	]),
	catch(
		call(CallbackGoal, UpdateTerm),
		Exc,
		log_error_and_fail(lang(Exc,
			watch_callback(CallbackGoal, UpdateTerm)))
	).

%%
watch_filter(triple(S,P,V), DB, Coll, WatchFilter) :-
	% TODO: howto handle scope of facts here?
	% TODO: support other terms here
	mng_get_db(DB, Coll, 'triples'),
	lang_triple:mng_triple_doc(triple(S,P,V), Doc, []),
	% the event holds the updated document in a field "fullDocument"
	% NOTE: not for remove events!
	maplist([[Key0,Val],[Key1,Val]]>>
		atom_concat('fullDocument.', Key0, Key1),
		Doc, WatchFilter).

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
:- once((setting(mng_client:read_only, true) ; initialize_one_db)).

