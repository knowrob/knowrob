:- module(lang_is_a,
    [ is_a(r,r),          % +Resource, ?Type
      instance_of(r,r),   % ?Individual, ?Class
      subclass_of(r,r),   % ?Class, ?SuperClass
      subproperty_of(r,r) % ?Property, ?SuperProperty
    ]).
/** <module> Type checking predicates.

@author Daniel Beßler
@license BSD
*/

:- op(1000, xfx, user:is_a).

:- use_module(library('model/OWL'),
    [ is_class/1,
      is_individual/1
    ]).

%% is_a(+Resource,?Type) is nondet.
%
% Wrapper around instance_of, subclass_of, and subproperty_of.
% Using this is a bit slower as an additional type check
% is needed.
% For example: `Cat is_a Animal` and `Nibbler is_a Cat`.
% 
% Note that contrary to wrapped predicates, is_a/2 requires
% the Resource to be ground.
%
% @param Resource a RDF resource
% @param Type the type of the resource
%

%is_a(A,_B) ?+>
%	var(A),
%	!,
%	throw(error(instantiation_error, is_a(resource_is_var))).
  
is_a(A,B) ?>
	ground(A),
	is_individual(A),
	!,
	instance_of(A,B).

is_a(A,B) ?>
	ground(A),
	is_class(A),
	!,
	subclass_of(A,B).

is_a(A,B) ?>
	ground(A),
	is_property(A),
	!,
	subproperty_of(A,B).

%% instance_of(?Entity,?Type) is nondet.
%
% The type of an entity (rdf:type).
% For example: `Nibbler instance_of Cat`.
%
% Note: that the *tell* clause of this rule allows
% Entity to be a variable, in which case a new entity
% symbol is generated.
%
% @param Entity a named individual
% @param Type the type of the entity
%

%instance_of(A,B) ?+>
%	pragma(is_list(B)),
%	instance_of_all(A,B).

instance_of(A,B) ?+>
	triple(A,rdf:type,B).

%% subclass_of(?Class,?SuperClass) is nondet.
%
% The subclass-of relation (rdfs:subClassOf).
% For example: `Cat subclass_of Animal`.
%
% @param Class a class IRI
% @param SuperClass a class IRI
%
subclass_of(A,B) ?+>
	% avoid that class description terms are passed to tripledb lookup
	% FIXME: but what about e.g. foo->V
	pragma(\+ compound(A)),
	pragma(\+ compound(B)),
	triple(A, rdfs:subClassOf, B).

%% subproperty_of(?Property,?SuperProperty) is nondet.
%
% The subproperty-of relation (rdfs:subPropertyOf).
%
% @param Property a property IRI
% @param SuperProperty a property IRI
%
subproperty_of(A,B) ?+>
	triple(A, rdfs:subPropertyOf, B).

%%
% Obtain IRI not yet used by any resource.
%
%unique_name(Type,Name) :-
%  once((Type=[Type_IRI|_] ; Type_IRI=Type)),
%  unique_name1(Type_IRI,Name).
%
%unique_name1(Type_IRI, Name) :-
%  % generate 8 random alphabetic characters
%  randseq(8, 25, Seq_random),
%  maplist(plus(65), Seq_random, Alpha_random),
%  atom_codes(Sub, Alpha_random),
%  % TODO: what IRI prefix? Currently we re-use the one of the type.
%  %        but that seems not optimal. Probably best to
%  %        have this in query context, and some meaningful default.
%  atomic_list_concat([Type_IRI,'_',Sub], IRI),
%  % check if there is no triple with this identifier as subject or object yet
%  ( is_resource(IRI) ->
%    unique_name(Type_IRI,Name);
%    Name = IRI
%  ).

     /*******************************
     *          UNIT TESTS          *
     *******************************/

:- begin_rdf_tests(
    lang_is_a,
    'package://knowrob/owl/test/swrl.owl',
    [ namespace('http://knowrob.org/kb/swrl_test#')
    ]).

test("ask and tell woman is a person") :-
  assert_true(is_a(test:'Woman', test:'TestThing')),
  assert_false(is_a(test:'Woman', test:'Person')),
  assert_true(tell(is_a(test:'Woman', test:'Person'))),
  assert_true(is_a(test:'Woman', test:'Person')).

test("ask and tell instances of Rex") :-
  assert_true(instance_of(test:'Rex', test:'Man')),
  assert_false(instance_of(test:'Rex', test:'Adult')),
  assert_true(tell(instance_of(test:'Rex', test:'Adult'))),
  assert_true(instance_of(test:'Rex', test:'Adult')).

test("ask and tell list instance of Greta") :-
  assert_false(instance_of('Greta', [test:'Woman', dul:'Person'])),
  assert_true(tell(instance_of('Greta', [test:'Woman', dul:'Person']))),
  assert_true(ask(instance_of_all('Greta', [test:'Woman', dul:'Person']))),
  assert_true(ask(instance_of('Greta', [test:'Woman', dul:'Person']))).

test("ask and tell sub property of") :-
  assert_true(subproperty_of(test:'hasParent', test:'hasAncestor')),
  assert_false(subproperty_of(test:'hasBrother', test:'hasSibling')),
  assert_true(tell(subproperty_of(test:'hasBrother', test:'hasSibling'))),
  assert_true(subproperty_of(test:'hasBrother', test:'hasSibling')).

% expect instantiation error as is_a expects a ground variable
test("ask _ is a Person", [throws(error(instantiation_error, _))]) :-
  is_a(_, dul:'Person').

:- end_rdf_tests(lang_is_a).
