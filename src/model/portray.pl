:- module(model_portray, []).

:- use_module(library('semweb/rdf_db'),
    [ rdf_current_ns/2,
      rdf_split_url/3
    ]).

user:portray(Term) :-
  convert_(Term,Term0),
  write(Term0).

convert_(Term,Term0) :-
  compound(Term),!,
  Term=..[Fun|Args],
  findall(Arg0, (
    member(Arg1,Args),
    convert_(Arg1,Arg0)
  ),Args0),
  Term0=..[Fun|Args0].

convert_(A,B) :-
  (atom(A);string(A)),
  atom_concat('_',_,A),
  ask(has_description(A,B0)),!,
  convert_(B0,B).

convert_(A,:(Prefix,NameEnquoted)) :-
  atom(A),
  rdf_split_url(URI,Name,A),
  rdf_current_ns(Prefix,URI),
  atom_chars(Name,[HeadC|_]),
  ( char_type(HeadC,lower) ->
    NameEnquoted=Name ;
    atomic_list_concat(['\'',Name,'\''],'',NameEnquoted)
  ),!.

convert_(A,A).
