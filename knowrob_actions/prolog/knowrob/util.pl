
:- module(util, [
     random_id/1,
     append_dl/3,
     write_concept/1
]).

random_id(Id) :-
  randseq(8, 25, Seq_random),
  maplist(plus(65), Seq_random, Alpha_random),
  atom_codes(Id, Alpha_random).

append_dl(Xs-Ys,Ys-Zs,Xs-Zs).

% FIXME: not sure why this is needed. write should display simplified
%        terms anyway. But it doesn't (always). What's the reason?
write_concept(Concept) :-
  rdf_has_prolog(Concept,rdfs:label,Label),!,
  write(''''), write(Label), write('''').
write_concept(Concept) :-
  rdf_split_url(_,Name,Concept),!,
  write(''''), write(Name), write('''').
  