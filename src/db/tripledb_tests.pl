:- module(tripledb_tests,
    [ begin_tripledb_tests/3,
      begin_tripledb_tests/2,
      end_tripledb_tests/1
    ]).
/** <module> A test environment for running tests using the tripledb.

@author Daniel Beßler
@license BSD
*/

:- use_module('./tripledb.pl',
	[ tripledb_load/2,
	  tripledb_forget/3
	]).

%%
begin_tripledb_tests(Name,RDFFile,Options0) :-
	( select_option(namespace(URI_Prefix),Options0,Options1)
	-> ( true )
	;  ( atom_concat(RDFFile,'#',URI_Prefix),
	     Options1=Options0 )
	),
	%%
	Setup   = tripledb_tests:tripledb_setup(RDFFile),
	Cleanup = tripledb_tests:tripledb_cleanup(URI_Prefix),
	add_option_goal_(Options1,setup(Setup),Options2),
	add_option_goal_(Options2,cleanup(Cleanup),Options3),
	%%
	begin_tests(Name,Options3),
	rdf_db:rdf_register_prefix(test,URI_Prefix,[force(true)]).

begin_tripledb_tests(Name,RDFFile) :-
	begin_tripledb_tests(Name,RDFFile,[]).

%%
end_tripledb_tests(Name) :-
	end_tests(Name).

%%
tripledb_setup(RDFFile) :-
	tripledb:set_default_graph(test),
	tripledb_load(RDFFile).

%%
tripledb_cleanup(URI_Prefix) :-
	wildcard_scope(QScope),
	tripledb_forget(_,_,_,QScope,[graph(test)]),
	tripledb:set_default_graph(user).

%%
add_option_goal_(OptionsIn,NewOpt,[MergedOpt|Rest]) :-
	NewOpt =.. [Key,NewGoal],
	OldOpt =.. [Key,OtherGoal],
	MergedOpt =.. [Key,Goal],
	( select_option(OldOpt,OptionsIn,Rest)
	-> ( Goal=(','(OtherGoal,NewGoal)) )
	;  ( Goal=NewGoal, Rest=OptionsIn )
	).
