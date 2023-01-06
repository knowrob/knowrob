:- module(rdf_test,
	[ begin_rdf_tests/3,
	  begin_rdf_tests/2,
	  end_rdf_tests/1
	]).
/** <module> A test environment for running tests with RDF data.
A RDF file can be loaded in the set-up step, and all assertions
written into a special named graph that is deleted again in cleanup step.

Example:
```
:- begin_rdf_tests('my_module', 'foo.rdf').

test('some test') :- fail.

:- end_tests('my_module').
```

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db'),
	[ rdf_unload_graph/1
	]).
:- use_module(library('semweb_ext'),
	[ ontology_url_graph/2,
	  load_rdf_xml/2,
	  set_default_graph/1,
	  get_subgraphs/2
	]).

%% begin_rdf_tests(+Name, +RDFFile, +Options) is det.
%
% Begin unit testing code section with RDF data.
% Load the RDF data during setup, and unload all asserted facts
% and RDF file on cleanup.
% Calls internally begin_tests/2.
%
%
begin_rdf_tests(Name,RDFFile,Options0) :-
	(	select_option(namespace(URI_Prefix),Options0,Options1)
	->	true
	;	(	atom_concat(RDFFile,'#',URI_Prefix),
			Options1=Options0
		)
	),
	%%
	Setup   = rdf_test:setup(RDFFile),
	Cleanup = rdf_test:cleanup(RDFFile),
	add_option_goal(Options1, setup(Setup), Options2),
	add_option_goal(Options2, cleanup(Cleanup), Options3),
	%%
	begin_tests(Name,Options3),
	rdf_db:rdf_register_prefix(test,URI_Prefix,[force(true)]).

%% begin_rdf_tests(+Name, +RDFFile) is det.
%
% Same as begin_rdf_tests/3 with empty options list.
%
begin_rdf_tests(Name,RDFFile) :-
	begin_rdf_tests(Name,RDFFile,[]).

%% end_rdf_tests(+Name) is det.
%
% End unit testing code section with RDF data.
%
end_rdf_tests(Name) :-
	end_tests(Name).

%%
setup(RDFFile) :-
	set_default_graph(test),
	load_rdf_xml(RDFFile,test).

%%
cleanup(RDFFile) :-
	cleanup,
	ontology_url_graph(RDFFile, OntoGraph),
	rdf_unload_graph(OntoGraph).

%%
cleanup :-
	get_subgraphs(test,Subs),
	forall(
		member(string(Sub),Subs),
		rdf_unload_graph(Sub)
	),
	set_default_graph(user).

%%
add_option_goal(OptionsIn,NewOpt,[MergedOpt|Rest]) :-
	NewOpt =.. [Key,NewGoal],
	OldOpt =.. [Key,OtherGoal],
	MergedOpt =.. [Key,Goal],
	(	select_option(OldOpt,OptionsIn,Rest)
	->	( Goal=(','(OtherGoal,NewGoal)) )
	;	( Goal=NewGoal, Rest=OptionsIn )
	).