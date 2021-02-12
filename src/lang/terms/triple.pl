:- module(lang_triple,
    [ triple(r,r,t) % ?Subject, ?Predicate, ?Object
    ]).
/** <module> The *triple* predicate.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('http/json')).
%% triple(?Subject, ?Property, ?Value) is nondet.
%
% Query values of a property on some subject in the triple DB.
%
% @param Subject The subject of a triple.
% @param Property The predicate of a triple.
% @param Value The object of a triple.
%
triple(Subject,Property,Value) ?>
	% validate input
	{ (atom(Subject);var(Subject))
	-> true
	;  throw(error(resource_error(Subject), ask(triple(Subject,Property,Value))))
	},
	% unpack options and scope
	options(Options),
	query_scope(QScope),
	fact_scope(FScope),
	% query the triple DB
	{ tripledb_ask(Subject,Property,Value,QScope,FScope,Options)
	}.

triple(Subject,Property,Value) +>
	% validate input
	{ atom(Subject)
	-> true
	;  throw(error(resource_error(Subject), tell(triple(Subject,Property,Value))))
	},
	% unpack options and scope
	options(Options),
	fact_scope(QScope),
	{ tripledb_tell(Subject,Property,Value,QScope,Options)
	}.

%% aggregate(?Triples) is nondet.
%
% Query a conjunction of triples.
% This is substantially faster then calling triple/3 due to a aggregate
% query being generated for the conjunction.
% The *Triples* list is a list holding terms `triple(S,P,O)`.
%
% @Triples a list of triple terms
%
aggregate(Triples) ?>
	% unpack options and scope
	options(Options),
	query_scope(QScope),
	fact_scope(FScope),
	% query the triple DB
	{ tripledb_aggregate(Triples,QScope,FScope,Options)
	}.

%% reads json data and asserts into mongodb
%
% @param FilePath - Path to the json file
%

triple_import_json(FilePath) :-
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
	tripledb_tell(S_atom, P_atom, O_value, Scope).

assert_triple_data(TriplesList) :-
    %handle case when given triples are list
    is_list(TriplesList),!,
    forall(member(X,TriplesList), assert_triple_data(X)).

%% triple_json_object(+Dict,-O) is semidet.
triple_json_object(Dict,O) :-
    % if Dict is dictionary
    is_dict(Dict),!,
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
	% check if given 'Since' and 'Until' are numbers
	(number(Since) -> true; throw(type_error(number, Since))),
	(number(Until) -> true; throw(type_error(number, Until))),
	time_scope(Since, Until, Scope).

triple_json_scope(_Triples,Scope) :-
    % create universal scope when either of 'since' or 'until' are not provided in triple
    universal_scope(Scope).

     /*******************************
     *	    UNIT TESTS	     		    *
     *******************************/

:- begin_tripledb_tests(
    'lang_triple',
    'package://knowrob/owl/test/swrl.owl',
    [ namespace('http://knowrob.org/kb/swrl_test#')
    ]).

test('aggregate simple') :-
	assert_true(ask(aggregate([
		triple(_,owl:onProperty,_)
	]))).

test('aggregate query') :-
	findall([R0,P0,M0,O0],
		(	triple(R0,owl:onProperty,P0),
			triple(R0,owl:minQualifiedCardinality,M0),
			triple(R0,owl:onClass,O0)
		),
		ResultsNonAggregate
	),
	findall([R1,P1,M1,O1],
		(	ask(aggregate([
				triple(R1,owl:onProperty,P1),
				triple(R1,owl:minQualifiedCardinality,M1),
				triple(R1,owl:onClass,O1)
			]))
		),
		ResultsAggregate
	),
	assert_true(ResultsAggregate \= []),
	forall(
		member(X,ResultsNonAggregate),
		assert_true(member(X,ResultsAggregate))
	).

%:- rdf_meta(measure_time(t)).
%measure_time(Goal) :-
%	time(forall(
%		between(1,10,_),
%		forall(
%			call(Goal),
%			true
%		)
%	)).
%
%test('time without aggregate1') :-
%	measure_time(
%		(	triple(R,owl:onProperty,_),
%			triple(R,owl:minQualifiedCardinality,_),
%			triple(R,owl:onClass,_)
%		)
%	).
%
%test('time with aggregate') :-
%	measure_time(ask(aggregate([
%			triple(R,owl:onProperty,_),
%			triple(R,owl:minQualifiedCardinality,_),
%			triple(R,owl:onClass,_)
%		]))).
  
:- end_tripledb_tests('lang_triple').
