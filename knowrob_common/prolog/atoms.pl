/** <module> Prolog/OWL utility predicates

  Copyright (C) 2016 Daniel Beßler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@author Daniel Beßler
@license BSD

*/

:- module(atoms,
    [ camelcase/2,
      lower_camelcase/2
]).

:- use_module(library('clpfd')). % TODO: what is this used for?
:- use_module(library('delay')). % TODO: what is this used for?

%% camelcase(?Underscore:atom, ?CamelCase:atom) is det.
%
%  For example, `camelcase(hello_world, 'HelloWorld')`. Works in both
%  directions.
camelcase(Underscore,CamelCase) :-
    when((ground(U0);ground(Underscore)),prepend_underscore(Underscore, U0)),
    delay(atom_codes(U0,U0Codes)),
    delay(atom_codes(CamelCase,CCodes)),
    once(camelcase_(U0Codes,CCodes)).
camelcase_([],[]).
camelcase_([0'_,Lower|Lowers], [Upper|Uppers]) :-
    upper_lower(Upper, Lower),
    camelcase_(Lowers, Uppers).
camelcase_([Lower|Lowers], [Lower|Uppers]) :-
    upper_lower(_,Lower),
    camelcase_(Lowers, Uppers).

%% lower_camelcase(?Underscore:atom, ?LowerCamelCase:atom) is det.
%
%  For example, `lower_camelcase(hello_world, helloWorld)`. Works in both
%  directions.
lower_camelcase(Underscore,LowerCamelCase) :-
    camelcase(Underscore,CamelCase),
    atom_codes(CamelCase,[Head|Tail]),
    Downcase is Head + 32,
    atom_codes(LowerCamelCase,[Downcase|Tail]).

%% upper_lower(?Upper:integer, ?Lower:integer) is semidet.
%
%  True if Upper (uppercase code) and Lower (lowercase code) represent the same letter.
%  Useful for bidirection case conversion among ASCII characters.
upper_lower(U,L) :-
    U #>= 0'A, U #=< 0'Z,
    L #>= 0'a, L #=< 0'z,
    L #= U + 32.

%% prepend_underscore(?Without:atom, ?With:atom) is semidet.
%
%  True if With has the same content as Without but the former starts
%  with an underscore character.
prepend_underscore(Without, With) :-
    delay(atom_codes(Without,WithoutCodes)),
    delay(atom_codes(With,WithCodes)),
    append([0'_],WithoutCodes,WithCodes).
