
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

%%
json_prolog_query(Goal, Answer) :-
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

json_prolog_encode(Number,Number) :-
  number(Number), !.

json_prolog_encode(String,String) :-
  string(String), !.

json_prolog_encode(Atom,Atom) :-
  atom(Atom), !.

%json_prolog_encode(Atom,Atom_json) :-
  %atom(Atom), !,
  %atomic_list_concat(['\'',Atom,'\''], '', Atom_json), !.

json_prolog_encode(Term,_{term: [Functor|Args_json]}) :-
  compound(Term), !,
  Term =.. [Functor|Args_pl],
  json_prolog_encode(Args_pl,Args_json).
  
