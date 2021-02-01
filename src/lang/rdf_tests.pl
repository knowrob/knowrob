:- module(rdf_tests,
	[ begin_rdf_tests/3,
	  begin_rdf_tests/2,
	  end_rdf_tests/1
	]).
/** <module> A test environment for running tests with RDF data.
A RDF file can be loaded in the set-up step, and all assertions
written into a special named graph that is deleted again in cleanup step.

@author Daniel Beßler
@license BSD
*/

:- use_module('db',
	[ load_owl/1,
	  drop_graph/1
	]).

%%
begin_rdf_tests(Name,RDFFile,Options0) :-
	(	select_option(namespace(URI_Prefix),Options0,Options1)
	->	true
	;	(	atom_concat(RDFFile,'#',URI_Prefix),
			Options1=Options0
		)
	),
	%%
	Setup   = rdf_tests:setup(RDFFile),
	Cleanup = rdf_tests:cleanup(RDFFile),
	add_option_goal(Options1, setup(Setup), Options2),
	add_option_goal(Options2, cleanup(Cleanup), Options3),
	%%
	begin_tests(Name,Options3),
	rdf_db:rdf_register_prefix(test,URI_Prefix,[force(true)]).

begin_rdf_tests(Name,RDFFile) :-
	begin_rdf_tests(Name,RDFFile,[]).

%%
end_rdf_tests(Name) :-
	end_tests(Name).

%%
setup(RDFFile) :-
	lang_query:set_default_graph(test),
	load_owl(RDFFile,[graph(test)]).

%%
cleanup(RDFFile) :-
	cleanup,
	lang_db:ontology_graph(RDFFile, OntoGraph),
	drop_graph(OntoGraph).

%%
cleanup :-
	drop_graph(test),
	lang_query:set_default_graph(user).

%%
add_option_goal(OptionsIn,NewOpt,[MergedOpt|Rest]) :-
	NewOpt =.. [Key,NewGoal],
	OldOpt =.. [Key,OtherGoal],
	MergedOpt =.. [Key,Goal],
	(	select_option(OldOpt,OptionsIn,Rest)
	->	( Goal=(','(OtherGoal,NewGoal)) )
	;	( Goal=NewGoal, Rest=OptionsIn )
	).