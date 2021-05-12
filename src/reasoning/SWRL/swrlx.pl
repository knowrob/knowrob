:- module(swrlx,
		[ swrlx_make_individual/2
		]).
/** <module> Prolog-based SWRL representation.

*/

:- use_module(library('lang/mongolog/database'),
		[ mongolog_add_predicate/3 ]).
:- use_module(swrl).

% add a database predicate that stores a mapping between
% individual IRI and a SWRL pattern of the rule that has
% generated the individual.
:- mongolog_add_predicate(swrlx_individual, [individual,pattern], [[pattern]]).

% declare some more clauses of the multifile predicate swrl_builtin/3
swrl:swrl_builtin(
		makeOWLIndividual, [A|Args],
		swrlx_make_individual(A_atom, [Label|Pattern]), Vars) :-
	memberchk(var('swrl:label',Label), Vars),
	swrl:swrl_atoms([A|Args], [A_atom|Pattern], Vars).

%
swrlx_make_individual1(Individual, Pattern) ?>
	% generate a unique IRI, use Type as prefix
	new_iri(Individual, owl:'Thing'),
	% assert facts about the new individual
	project(is_individual(Individual)),
	project(has_type(Individual, owl:'Thing')),
	% assert mapping between pattern and individual
	assert(swrlx_individual(Individual, Pattern)).

%%
%
% If the first argument is already bound when make_individual/3 is called,
% this method returns true and no individual is created
%
swrlx_make_individual(Individual, Pattern) ?>
	ground(Pattern),
	atomic_list_concat(Pattern, '::', PatternAtom),
	once((
		% succeed if individual is an atom already
		atom(Individual)
		% find existing individual given the pattern
	;	(var(Individual), swrlx_individual(Individual, PatternAtom))
		% else create a new individual
	;	(var(Individual), swrlx_make_individual1(Individual, PatternAtom))
	)).

:- begin_rdf_tests(
		'swrlx',
		'package://knowrob/owl/test/swrl.owl',
		[ namespace('http://knowrob.org/kb/swrl_test#')
		]).

% % % % % % % % % % % % % % % % % % % % % % % %
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
	% FIXME: has_individual_pattern/2 facts are not deleted in cleanup step!
	%        there should be a mechanism to do this under the hood.
	forall(
		( member(X,Cars1), \+ member(X,Cars0) ),
		kb_call(retractall(swrlx_individual(X,_)))
	).

:- end_rdf_tests('swrlx').
