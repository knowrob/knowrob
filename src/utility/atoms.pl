:- module(utils_atoms,
    [ lowercase/2,
      camelcase/2,
      lower_camelcase/2,
      atom_ensure_prefix/3,
      atom_ensure_suffix/3,
      atom_remove_prefix/3
    ]).
/** <module> Prolog/OWL utility predicates

@author Daniel Beßler
@license BSD
*/

:- use_module(library(clpfd)).
:- use_module(library(delay)).

%% atom_ensure_prefix(?Atom:atom, ?Prefix:atom, -AtomResolved:atom) is semidet.
atom_ensure_prefix(Atom, Prefix, Atom) :-
  sub_atom(Atom, 0, _, _, Prefix), !.
atom_ensure_prefix(Atom, Prefix, AtomResolved) :-
  atom_concat(Prefix, Atom, AtomResolved).

%% atom_ensure_suffix(?Atom:atom, ?Suffix:atom, -AtomResolved:atom) is semidet.
atom_ensure_suffix(Atom, Suffix, Atom) :-
  atom_concat(_, Suffix, Atom), !.
atom_ensure_suffix(Atom, Suffix, AtomResolved) :-
  atom_concat(Atom, Suffix, AtomResolved).

%% atom_remove_prefix(?Atom:atom, ?Prefix:atom, -AtomResolved:atom) is semidet.
atom_remove_prefix(Atom, Prefix, AtomOut) :-
  atom_concat(Prefix, AtomOut, Atom), !.
atom_remove_prefix(Atom, _, Atom).

%% lowercase(?Upper:atom, ?Lower:atom) is semidet.
lowercase(Upper,Lower) :-
    (ground(Upper);ground(Lower)),
    delay(atom_codes(Upper,UpperCodes)),
    delay(atom_codes(Lower,LowerCodes)),
    lowercase_(LowerCodes, UpperCodes).
lowercase_([],[]).
lowercase_([Lower|Lowers], [Upper|Uppers]) :-
    upper_lower(Upper, Lower),
    lowercase_(Lowers, Uppers), !.
lowercase_([C|Lowers], [C|Uppers]) :- lowercase_(Lowers, Uppers).

%% camelcase(?Underscore:atom, ?CamelCase:atom) is det.
%
%  For example, `camelcase(hello_world, 'HelloWorld')`. Works in both
%  directions.
camelcase(Underscore,CamelCase) :-
    compound(Underscore),
    term_to_atom(Underscore, Underscore_),
    camelcase(Underscore_,CamelCase), !.
camelcase(Underscore,CamelCase) :-
    compound(CamelCase),
    term_to_atom(CamelCase, CamelCase_),
    camelcase(Underscore,CamelCase_), !.
camelcase(Underscore,CamelCase) :-
    when((ground(U0);ground(Underscore)),prepend_underscore(Underscore, U0)),
    delay(atom_codes(U0,U0Codes)),
    delay(atom_codes(CamelCase,CCodes)),
    once(camelcase_(U0Codes,CCodes)).
camelcase_([],[]).
camelcase_([0'-,Lower|Lowers], [0'-,Upper|Uppers]) :-
    upper_lower(Upper, Lower),
    camelcase_(Lowers, Uppers), !.
camelcase_([0'_,Lower|Lowers], [Upper|Uppers]) :-
    Upper #>= 0'A, Upper #=< 0'Z,
    upper_lower(Upper, Lower),
    camelcase_(Lowers, Uppers).
camelcase_([Lower|Lowers], [Lower|Uppers]) :-
    upper_lower(_,Lower),
    camelcase_(Lowers, Uppers).

%% lower_camelcase(?Underscore:atom, ?LowerCamelCase:atom) is det.
%
%  For example, `lower_camelcase(hello_world, helloWorld)`. Works in both
%  directions.
lower_camelcase(Underscore,CamelCase) :-
    compound(Underscore),
    term_to_atom(Underscore, Underscore_),
    lower_camelcase(Underscore_,CamelCase), !.
lower_camelcase(Underscore,CamelCase) :-
    compound(CamelCase),
    term_to_atom(CamelCase, CamelCase_),
    lower_camelcase(Underscore,CamelCase_), !.
lower_camelcase(Underscore,LowerCamelCase) :-
    (  nonvar(Underscore)
    -> (
        camelcase(Underscore,CamelCase),
        atom_codes(CamelCase,[Head|Tail]),
        Downcase is Head + 32,
        atom_codes(LowerCamelCase,[Downcase|Tail])
    ) ; (
        nonvar(LowerCamelCase),
        atom_codes(LowerCamelCase,[Head|Tail]),
        Upcase is Head - 32,
        atom_codes(CamelCase,[Upcase|Tail]),
        camelcase(Underscore,CamelCase)
    )).

%% upper_lower(?Upper:integer, ?Lower:integer) is semidet.
%
%  True if Upper (uppercase code) and Lower (lowercase code) represent the same letter.
%  Useful for bidirection case conversion among ASCII characters.
upper_lower(0'-,0'-) :- !.
upper_lower(U,L) :-
    U #>= 0'A, U #=< 0'Z,
    L #>= 0'a, L #=< 0'z,
    L #= U + 32, !.
upper_lower(U,L) :-
    ground(U),
    U >= 0'a, U =< 0'z,
    L is U, !.
upper_lower(U,L) :-
    ground(L),
    L >= 0'A, L =< 0'Z,
    U is L, !.

%% prepend_underscore(?Without:atom, ?With:atom) is semidet.
%
%  True if With has the same content as Without but the former starts
%  with an underscore character.
prepend_underscore(Without, With) :-
    delay(atom_codes(Without,WithoutCodes)),
    delay(atom_codes(With,WithCodes)),
    append([0'_],WithoutCodes,WithCodes).
