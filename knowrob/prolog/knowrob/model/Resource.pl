
:- module('knowrob/model/Resource',
    [
      kb_resource/1
    ]).

:- use_module(library('semweb/rdf_db')).

:- rdf_meta kb_resource(t).

%% kb_resource(+Res) is semidet.
%
% Test wether the resource provided in the argument
% is known in the knowledge base.
% A warning is printed if this is not the case.
%
kb_resource(Res) :-
  \+ atom(Res), !,
  print_message(warning, rdf_error(not_an_atom(Res))),
  fail.

kb_resource(Res) :-
  ( rdf_has(Res,rdf:type,_);
    rdf_equal(Res,rdf:type)
  ),!.

kb_resource(Res) :-
  print_message(warning, rdf_error(unknown(Res))),
  fail.
