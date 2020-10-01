:- module(lang_triple,
    [ triple(r,r,t) % ?Subject, ?Predicate, ?Object
    ]).
/** <module> The *triple* predicate.

@author Daniel Beßler
@license BSD
*/

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

     /*******************************
     *	    UNIT TESTS	     		    *
     *******************************/

:- begin_tripledb_tests(
    'lang_triple',
    'package://knowrob/owl/test/swrl.owl',
    [ namespace('http://knowrob.org/kb/swrl_test#')
    ]).

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
