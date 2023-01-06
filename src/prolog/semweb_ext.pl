:- module(semweb_ext,
    [ register_ontology_namespace/2,
      ontology_url/4,
      ontology_url_graph/2,
      ontology_url_version/2,
      ontology_url_read/2,
      add_subgraph/2,
      get_supgraphs/2,
      get_subgraphs/2,
      set_default_graph/1,
      set_graph_option/2,
      load_json_rdf/1,
      load_rdf_xml/2
    ]).
/** <module> Extensions around the semweb module of Prolog.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('logging')).
:- use_module(library('rdf'),
		[ load_rdf/3 ]).
:- use_module(library('semweb/rdf_db'), 
		[ rdf_equal/2, rdf_register_ns/3, rdf_load/2, rdf/4 ]).
:- use_module(library('http/http_open'),
		[ http_open/3 ]).
:- use_module(library('url'),
		[ url_resolve/2 ]).
:- use_module(library('xsd'),
		[ xsd_data_basetype/2 ]).
:- use_module(library('scope'),
		[ universal_scope/1 ]).
:- use_module(library(dcg/basics)).

%%
annotation_property('http://www.w3.org/2000/01/rdf-schema#comment').
annotation_property('http://www.w3.org/2000/01/rdf-schema#seeAlso').
annotation_property('http://www.w3.org/2000/01/rdf-schema#label').
annotation_property('http://www.w3.org/2002/07/owl#versionInfo').

%% register_ontology_namespace(+URL,+Options) is semidet.
%
% @param URL URL of a RDF file.
% @param Options List of options.
%
register_ontology_namespace(URL, Opts) :-
	% register namespace
	(	option(namespace(NS), Opts)
	->	(	atom_concat(URL, '#', Prefix),
			rdf_register_ns(NS, Prefix, [keep(true)])
		)
	;	true
	),
	(	option(namespace(NS,Prefix), Opts)
	->	rdf_register_ns(NS, Prefix, [keep(true)])
	;	true
	).

%%
%
ontology_url(URL, Resolved, OntologyGraph, OntologyVersion) :-
	(	url_resolve(URL,Resolved)
	->	log_debug(db(url_resolved(URL,Resolved)))
	;	Resolved=URL 
	),
	ontology_url_version(Resolved, OntologyVersion),
	ontology_url_graph(Resolved, OntologyGraph).

%%
% each ontology is stored in a separate graph named
% according to the ontology
%
ontology_url_graph(URL,Name) :-
	file_base_name(URL,FileName),
	file_name_extension(Name,_,FileName),
	!.

%%
%
%
ontology_url_read(URL, Opts) :-
	rdf_equal(owl:'Ontology',OWL_Ontology),
	rdf_equal(rdf:'type',RDF_Type),
	% load RDF data and ookup ontology URL
	load_rdf_(URL, Triples),
	(	member(rdf(AssertedURL,RDF_Type,OWL_Ontology), Triples) -> true
	;	log_error_and_fail(type_error(ontology,URL))
	),
	% convert to triple/3 and annotation/3 terms
	maplist(convert_rdf_(AssertedURL), Triples, Terms),
	% NOTE: annotations are stored in a separate collection.
	%       the reason is that we create a search index over the value
	%       of a triple, and that mongo cannot generate such an index
	%       over values with special characters.
	partition(is_annotation_triple(Terms), Terms,
		AnnotationTriples, TripleTerms),
	maplist([triple(S,P,O),annotation(S,P,O)]>>true,
		AnnotationTriples, AnnotationTerms),
	% unify options
	ignore(option(asserted_url(AssertedURL), Opts)),
	ignore(option(triples(TripleTerms), Opts)),
	ignore(option(annotations(AnnotationTerms), Opts)).

%%
is_annotation_triple(_, triple(_,P,_)) :-
	annotation_property(P),!.
is_annotation_triple(Terms, triple(_,P,_)) :-
	rdf_equal(rdf:'type',RDF_Type),
	rdf_equal(owl:'AnnotationProperty', AnnotationProperty),
	memberchk(triple(P, RDF_Type, string(AnnotationProperty)), Terms),!.

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


     /*******************************
      *       GRAPH HIERARCHY       *
      *******************************/

:- dynamic subgraph_of/2.

%% add_subgraph(+Sub,+Sup) is det.
%
% Adds the subgraph-of relation between two named graphs.
%
% @param Sub Name of the subgraph.
% @param Sup Name of the super-graph.
%
add_subgraph(Sub,Sup) :-
	get_supgraphs(Sup,SupGraphs),
	get_subgraphs(Sub,SubGraphs),
	forall(
		(	member(string(Sup_x),SupGraphs),
			member(string(Sub_x),SubGraphs)
		),
		assert_subgraph_of(Sub_x,Sup_x)
	).

%%
assert_subgraph_of(Sub,Sup) :- subgraph_of(Sub,Sup),!.
assert_subgraph_of(Sub,Sup) :- assertz(subgraph_of(Sub,Sup)).

%% get_supgraphs(+GraphName,-SupGraphs) is det.
%
% Get all super-graphs of a named graph. This minds transitivity of the relation.
%
% @param GraphName Name of a graph.
% @param SupGraphs List of super-graphs.
%
get_supgraphs(G,_) :-
	var(G),
	!.

get_supgraphs(G,Graphs) :-
	findall(string(X),
		(	X=G
		;	subgraph_of(G,X)
		),
		Graphs
	).

%% get_subgraphs(+GraphName,-SubGraphs) is det.
%
% Get all subgraphs of a named graph. This minds transitivity of the relation.
%
% @param GraphName Name of a graph.
% @param SubGraphs List of subgraphs.
%
get_subgraphs(G,Graphs) :-
	findall(string(X),
		(	X=G
		;	subgraph_of(X,G)
		),
		Graphs
	).

     /*******************************
     *          VERSIONING          *
     *******************************/

%% get modification time of local file.
%% this is to cause re-loading the file in case of it has changed locally.
ontology_url_version(URL, Version) :-
	catch(exists_file(URL), _, fail),
	!,
	set_time_file(URL, [modified(ModStamp)],[]),
	atom_number(Version, ModStamp).

%% try to extract version from URI
ontology_url_version(URL, Version) :-
	atomic_list_concat(XL,'/',URL),
	% TODO: some ontologies use e.g. ".../2005/07/xx.owl"
	reverse(XL, [_,Version|_]),
	is_version_string(Version),
	!.

%% remote URL -> reload only one update per day
ontology_url_version(_URL, Version) :-
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
      *    LOADING RDF/XML DATA     *
      *******************************/

%%
%
load_rdf_xml(URL, ParentGraph) :-
	rdf_equal(owl:'imports', OWL_Imports),
	rdf_equal(owl:'Ontology',OWL_Ontology),
	rdf_equal(rdf:'type',RDF_Type),
	% resolve URL, and read ontology graph and version
	ontology_url(URL, Resolved, OntologyGraph, OntologyVersion),
	% setup graph structure: parent - ontology - user
	(	add_subgraph(OntologyGraph,ParentGraph),
		add_subgraph(user,OntologyGraph)
	),
	% load RDF data
	rdf_load(Resolved, [graph(OntologyGraph), silent(true)]),
	% lookup ontology URL
	(	rdf(AssertedURL, RDF_Type, OWL_Ontology, OntologyGraph) -> true
	;	log_error_and_fail(type_error(ontology,URL))
	),
	% load RDF data of imported ontologies
	forall(
		rdf(AssertedURL, OWL_Imports, ImportedURL, OntologyGraph),
		load_rdf_xml(ImportedURL, ParentGraph)
	),
	!,
	log_debug(prolog(ontology_loaded(OntologyGraph,OntologyVersion))).

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
	% TODO: convert_typed_literal(:ConvertPred) can be used to convert literals
	%load_rdf(URL, Triples, [blank_nodes(noshare), namespaces(NSList)]).
	load_rdf(URL, Triples, [blank_nodes(noshare)]).

     /*******************************
     *          JSON DATA          *
     *******************************/

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

%% triple_json_object(+Dict,-Obj) is semidet.
triple_json_object(Dict,Obj) :-
	% if the argument is a dictionary
	is_dict(Dict),!,
	get_dict('$numberDecimal', Dict, Json_Object),
	(	atom(Json_Object)   -> atom_number(Json_Object,Obj)
	;	string(Json_Object) -> number_string(Obj,Json_Object)
	;	Obj is Json_Object
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


		 /*******************************
		 *	    	OPTIONS		  	 	*
		 *******************************/

% name of default fact graph
:- dynamic default_graph/1.

default_graph(user).

%%
% Set the name of the graph where facts are asserted and retrieved
% if no other graph was specified.
%
set_default_graph(Graph) :-
	retractall(default_graph(_)),
	assertz(default_graph(Graph)).

%%
set_graph_option(Options, Options) :-
	option(graph(_), Options),
	!.
set_graph_option(Options, Merged) :-
	default_graph(DG),
	merge_options([graph(DG)], Options, Merged).
