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

:- rdf_meta(measure_time(t)).
measure_time(Goal) :-
	time(forall(
		between(1,10,_),
		forall(
			call(Goal),
			true
		)
	)).

test('without aggregate2') :-
	forall(
		(	triple(R,owl:onProperty,P),
			triple(R,owl:minQualifiedCardinality,M),
			triple(R,owl:onClass,O)
		),
		writeln([R,P,M,O])
	).

test('aggregate2') :-
	forall(ask(aggregate([
			triple(R,owl:onProperty,P),
			triple(R,owl:minQualifiedCardinality,M),
			triple(R,owl:onClass,O)
		])),
		writeln([R,P,M,O])
	).

test('without aggregate1') :-
	measure_time(
		(	triple(R,owl:onProperty,_),
			triple(R,owl:minQualifiedCardinality,_),
			triple(R,owl:onClass,_)
		)
	).

test('aggregate1') :-
	measure_time(ask(aggregate([
			triple(R,owl:onProperty,_),
			triple(R,owl:minQualifiedCardinality,_),
			triple(R,owl:onClass,_)
		]))).
  
:- end_tripledb_tests('lang_triple').
