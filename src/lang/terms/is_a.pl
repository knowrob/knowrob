:- module(lang_is_a,
    [ is_a(r,r),
      instance_of(r,r),
      subclass_of(r,r),
      subproperty_of(r,r),
      unique_name(r,-)
    ]).
/** <module> The *is_a* predicate.

@author Daniel Beßler
@license BSD
*/

:- op(1000, xfx, user:is_a).

:- use_module(library('comm/notify'),
    [ notify/1
    ]).
:- use_module(library('model/RDFS'),
    [ is_resource/1
    ]).
:- use_module(library('model/OWL'),
    [ is_class/1,
      is_individual/1
    ]).

%% is_a(?A,?B) is nondet.
%
% Wrapper around instance_of and subclass_of.
% Using this is a bit slower as an additional type check
% is needed at the moment.
% For example: `Cat is_a Animal` and `Nibbler is_a Cat`.
%
is_a(A,B) ?>
  { var(A); (is_individual(A),!) },
  instance_of(A,B).

is_a(A,B) ?>
  { ground(A), is_class(A), ! },
  subclass_of(A,B).

is_a(A,B) ?>
  { ground(A), is_property(A), ! },
  subproperty_of(A,B).

is_a(A,B) +>
  { is_class(A), ! },
  subclass_of(A,B).

is_a(A,B) +>
  { is_property(A), ! },
  subproperty_of(A,B).

is_a(A,B) +>
  { ( var(A); is_individual(A) ), ! },
  instance_of(A,B).

%% instance_of(?A,?B) is nondet.
%
% The type of an entity (rdf:type).
% For example: `Nibbler instance_of Cat`.
%
instance_of(A,B) ?> has_type(A,B).

instance_of(A,B) +>
  { owl_description(B,intersection_of(List)),! },
  instance_of_all(A,List).
instance_of(A,B) +>
  % generate a new name in case A is a variable
  { var(A), ! },
  { unique_name(B,A) },
  { is_class(B) ->
    tell(is_individual(A)) ;
    true },
  instance_of(A,B).
instance_of(A,B) +>
  triple(A,rdf:type,B),
  notify(individual(A)).

%%
instance_of_all(_S,[]) ?+> { true }.
instance_of_all(S,[First|Rest]) ?+>
  instance_of(S,First),
  instance_of_all(S,Rest).

%% subclass_of(?A,?B) is nondet.
%
% The subclass-of relation (rdfs:subClassOf).
% For example: `Cat subclass_of Animal`.
%
subclass_of(A,B) ?> { ground([A,B]), A=B, ! }.
subclass_of(A,B) ?+> triple(A,rdfs:subClassOf,B).

%% subproperty_of(?A,?B) is nondet.
%
% The subproperty-of relation (rdfs:subPropertyOf).
%
subproperty_of(A,B) ?> { ground([A,B]), A=B, ! }.
subproperty_of(A,B) ?+> triple(A,rdfs:subPropertyOf,B).

%% unique_name(+Type,-Name) is det.
%
% Obtain IRI not yet used by any resource.
%
unique_name(Type,Name) :-
  once((Type=[Type_IRI|_] ; Type_IRI=Type)),
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
    unique_name(Type_IRI,Name);
    Name = IRI
  ).
