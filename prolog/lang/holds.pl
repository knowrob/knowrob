
:- module('knowrob/lang/holds',
    [
      holds/1,         % ?Predicate(?Subject,?Object)
      holds/2,         % ?Predicate(?Subject,?Object), +Time
      holds/3,         % ?Subject, ?Predicate, ?Object
      holds/4          % ?Subject, ?Predicate, ?Object, +Time
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('knowrob/lang/ask'), [
    kb_triple/4
]).

:- rdf_meta holds(t),
            holds(t,?),
            holds(r,r,t),
            holds(r,r,t,?).

%% holds(+Term).
%% holds(+Term, ?Interval).
%% holds(?S, ?P, ?O).
%% holds(?S, ?P, ?O, ?Interval).
%
% True iff relation `P(S,O)` holds during Interval, and with Term=P(S,O).
% Where Interval is a TimeInterval or TimePoint individual,
% a number or a list of two numbers representing a time interval.
%
% @param Term Must be of the form: PROPERTY(SUBJECT, OBJECT).
% @param T Can be TimeInterval or TimePoint individual, a number or a list of two numbers representing a time interval.
%
holds(Term) :-
  holds(Term,now).

holds(Term, Time) :-
  var(Term),
  holds(S,P,O,Time),
  Term =.. [':', P, (S,O)].

holds(Term, Time) :-
  nonvar(Term),
  unwrap_triple(Term,S,P,O),
  holds(S,P,O,Time).

holds(S,P,O) :-
  holds(S,P,O,now).

holds(S,P,O,now) :-
  !, kb_triple(S,P,O,_{}).

holds(S,P,O,Time) :-
  kb_triple(S,P,O,_{during:Time}).

%%
unwrap_triple(Term,S,P,O) :-
  Term =.. [':', Namespace, Tail],!,
  Tail =.. [PX,S,O],
  % unpack namespace
  rdf_current_ns(Namespace, NamespaceUri),
  atom_concat(NamespaceUri, PX, P).

unwrap_triple(Term,S,P,O) :-
  Term =.. [P,S,O].
