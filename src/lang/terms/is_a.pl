:- module(lang_is_a,
    [ is_a(r,r),
      is_instance_of(r,r),
      is_subclass_of(r,r),
      unique_name(r,-),
      op(1000, xfx, is_a)
    ]).
/** <module> The *is_a* predicate.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('model/RDFS'),
        [ is_resource/1 ]).
:- use_module(library('model/OWL'),
        [ is_class/1, is_individual/1 ]).
:- use_module(library('db/tripledb'),
        [ tripledb_type_of/2, tripledb_subclass_of/2, tripledb_tell/4 ]).
:- use_module(library('reasoning/pool'),
        [ infer/2 ]).

%% is_a(?A,?B) is nondet.
%
% Wrapper around is_instance_of and is_subclass_of.
% Using this is a bit slower as an additional type check
% is needed at the moment.
% For example: `Cat is_a Animal` and `Nibbler is_a Cat`.
%
is_a(A,B) ?>
  { var(A); is_individual(A) },
  is_instance_of(A,B).

is_a(A,B) ?>
  { ground(A), is_class(A) },
  is_subclass_of(A,B).

is_a(A,B) +>
  { is_class(A), ! },
  is_subclass_of(A,B).

is_a(A,B) +>
  { ( var(A); is_individual(A) ), ! },
  is_instance_of(A,B).

%% is_instance_of(?A,?B) is nondet.
%
% The type of an entity (rdf:type).
% For example: `Nibbler is_instance_of Cat`.
%
is_instance_of(A,B) ?> { tripledb_type_of(A,B) }.
is_instance_of(A,B) ?> infer(is_instance_of(A,B)).
is_instance_of(A,B) +>
  % generate a new name in case A is a variable
  { var(A), ! },
  { unique_name(B,A) },
  { tell( is_individual(A) )},
  is_instance_of(A,B).
is_instance_of(A,B) +> tripledb_tell(A,rdf:type,B).

%% is_subclass_of(?A,?B) is nondet.
%
% The subclass-of relation (rdfs:subClassOf).
% For example: `Cat is_subclass_of Animal`.
%
is_subclass_of(A,B) ?> { tripledb_subclass_of(A,B) }.
is_subclass_of(A,B) ?> infer(is_subclass_of(A,B)).
is_subclass_of(A,B) +> tripledb_tell(A,rdfs:subClassOf,B).

%% unique_name(+Type,-Name) is det.
%
% Obtain IRI not yet used by any resource.
%
unique_name(Type,Name) :-
  get_type_iri(Type,Type_IRI),
  unique_name1(Type_IRI,Name).

unique_name1(Type_IRI, Name) :-
  % generate 8 random alphabetic characters
  randseq(8, 25, Seq_random),
  maplist(plus(65), Seq_random, Alpha_random),
  atom_codes(Sub, Alpha_random),
  % TODO: what IRI prefix? Currently we re-use the one of the type.
  %        but that seems not optimal. Probably best to
  %        have this in query context, and some meaningful default.
  atomic_list_concat([Type_IRI,'_',Sub], IRI),
  % check if there is no triple with this identifier as subject or object yet
  ( is_resource(IRI) ->
    unique_name(Type,Name);
    Name = IRI
  ).

%%
get_type_iri([X|_],IRI) :-
  get_type_iri(X,IRI).

get_type_iri(X,X).
