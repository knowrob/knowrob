/*  json_prolog.pl

    Author:        Daniel Beßler
    E-mail:        danielb@uni-bremen.de
    WWW:           http://www.ease-crc.org
    Copyright (C): 2019, University of Bremen

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

    As a special exception, if you link this library with other files,
    compiled with a Free Software compiler, to produce an executable, this
    library does not by itself cause the resulting executable to be covered
    by the GNU General Public License. This exception does not however
    invalidate any other reasons why the executable file might be covered by
    the GNU General Public License.
*/

:- module(json_prolog,
    [
      json_prolog_query/2,
      json_prolog_encode/2
    ]).
/** <module> json_prolog

@author Daniel Beßler
@license BSD
*/

:- use_module(library('http/json')).

%% json_prolog_query(+Goal, -Answer) is semidet.
%
% Calls *Goal*, which is an atom representation of
% a term, and JSON encodes the solution and binds
% it the the second argument *Answer*.
%
json_prolog_query(Goal, Answer) :-
  atom(Goal),
  % parse term from Goal atom
  read_term_from_atom(Goal,GoalTerm,[variable_names(Args)]),
  expand_goal(GoalTerm,Expanded),
  % call the goal
  call(Expanded),
  % create a dictionary
  findall(Name-JSON_value, (
    member(Name=Value,Args),
    json_prolog_encode(Value,JSON_value)
  ), Pairs),
  dict_pairs(Dict,_,Pairs),
  % convert to JSON 
  with_output_to(atom(Answer), 
    json_write_dict(current_output, Dict)
  ).

%%
json_prolog_encode([],[]) :- !.

json_prolog_encode([X|Xs],[Y|Ys]) :-
  json_prolog_encode(X,Y),
  json_prolog_encode(Xs,Ys),!.

json_prolog_encode(String,Atom) :-
  string(String),
  term_to_atom(String,Atom), !.

json_prolog_encode(X,X) :-
  ( atom(X) ; is_dict(X) ; number(X) ; string(X) ), !.

json_prolog_encode(Term,_{term: [Functor|Args_json]}) :-
  compound(Term), !,
  Term =.. [Functor|Args_pl],
  json_prolog_encode(Args_pl,Args_json).
  
