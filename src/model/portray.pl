:- module(model_portray, []).
/** <module> portray for language terms and RDF resources.

@author Daniel Beßler
*/

:- use_module(library('semweb/rdf_db'),
    [ rdf_current_ns/2,
      rdf_split_url/3
    ]).

%%
% user:portray clauses are used to display terms
% in a human readable fashion to the user.
%
user:portray(Term) :-
  convert_(Term,Term0),
  write(Term0).

convert_(Term,Term0) :-
  % recursively call convert_ on compound terms
  compound(Term),!,
  Term=..[Fun|Args],
  findall(Arg0, (
    member(Arg1,Args),
    convert_(Arg1,Arg0)
  ),Args0),
  Term0=..[Fun|Args0].

convert_(A,B) :-
  % read description for class names starting with underscore.
  % this is the case e.g. for OWL restriction classes
  (atom(A);string(A)),
  atom_concat('_',_,A),
  ask(has_description(A,B0)),
  B0 \= class(_),!,
  convert_(B0,B).

convert_(A,:(Prefix,NameEnquoted)) :-
  % convert full IRI to short notation if possible
  atom(A),
  rdf_split_url(URI,Name,A),
  rdf_current_ns(Prefix,URI),
  atom_chars(Name,[HeadC|_]),
  ( char_type(HeadC,lower) ->
    NameEnquoted=Name ;
    atomic_list_concat(['\'',Name,'\''],'',NameEnquoted)
  ),!.

convert_(A,A).
