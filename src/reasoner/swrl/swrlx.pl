:- module(swrlx,
		[ swrlx_make_individual/2
		]).
/** <module> Support for the SWRL Extensions built-in library.

@author Álvaro Páez
@author Daniel Beßler
@see https://github.com/protegeproject/swrlapi/wiki/ExtensionsBuiltInLibrary
*/

:- use_module(library('lang/mongolog/database'),
		[ mongolog_add_predicate/3 ]).
:- use_module(swrl).

% add a database predicate that stores a mapping between
% individual IRI and a SWRL pattern of the rule that has
% generated the individual.
%
:- mongolog_add_predicate(swrlx_individual, [individual,pattern], [[pattern]]).

% SWRL Extensions are implemented through additional clauses
% of the multifile predicate swrl_builtin/4.
%
swrl:swrl_builtin(
		makeOWLIndividual, [A|Args],
		swrlx_make_individual(A_atom, [Label|Pattern]),
		Vars) :-
	% read the label of the rule the builtin is embedded in
	memberchk(var('swrl:label',Label), Vars),
	swrl:swrl_atoms([A|Args], [A_atom|Pattern], Vars).

%
swrlx_make_new_individual(Individual, Pattern) ?>
	% generate a unique IRI, use Type as prefix
	new_iri(Individual, owl:'Thing'),
	% assert facts about the new individual
	project(is_individual(Individual)),
	project(has_type(Individual, owl:'Thing')),
	% assert mapping between pattern and individual
	assert(swrlx_individual(Individual, Pattern)).

%% swrlx_make_individual(?Individual, +Pattern) is det.
%
% This predicate provides a controlled way of creating OWL individuals in a rule.
% It succeeds in any case if Individual is instantiated before the call.
% Pattern is a signature of instantiations in a rule, and if the pattern was used
% before to create an individual, then Individual will be instantiated to
% the previously created individual.
%
% @param Individual IRI atom
% @param Pattern List of atoms
%
swrlx_make_individual(Individual, Pattern) ?>
	ground(Pattern),
	atomic_list_concat(Pattern, '::', PatternAtom),
	once((
		% succeed if individual is an atom already
		atom(Individual)
		% read indiviudal from cache
	;	(var(Individual), swrlx_individual(Individual, PatternAtom))
		% cache miss: create a new individual
	;	(var(Individual), swrlx_make_new_individual(Individual, PatternAtom))
	)).

:- begin_rdf_tests('swrlx', 'owl/test/swrl.owl').

:- sw_register_prefix(test, 'http://knowrob.org/kb/swrl_test#').

test(swrl_makeOWLIndividual) :-
	findall(X, has_type(X, test:'Car'), Cars0),
	swrl_file_path(knowrob,'test.swrl',Filepath),
	swrl_file_fire(Filepath,'makeOWLIndividual'),
	findall(X, has_type(X, test:'Car'), Cars1),
	swrl_file_fire(Filepath,'makeOWLIndividual'),
	findall(X, has_type(X, test:'Car'), Cars2),
	% firing the rule for the first time creates one new individual
	length(Cars0, NumCars0),
	length(Cars1, NumCars1),
	assert_true(NumCars1 is NumCars0+1),
	% firing the rule a second time does not change number of individuals
	assert_equals(Cars1, Cars2),
	% delete the fact again
	forall(
		( member(X,Cars1), \+ member(X,Cars0) ),
		kb_call(retractall(swrlx_individual(X,_)))
	).

:- end_rdf_tests('swrlx').
