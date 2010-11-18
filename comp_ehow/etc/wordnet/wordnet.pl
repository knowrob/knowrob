:-use_module(library('semweb/rdf_db')).
:-use_module(library('semweb/rdfs_computable')).
:-rdf_db:rdf_register_ns(wn20schema, 'http://www.w3.org/2006/03/wn/wn20/schema/').

%
% Checks if the given word Label belongs to a synset with ID SynsetId.
%
word_has_synset_id(Label, SynsetId):- 
  rdf_has(Synset, wn20schema:synsetId, literal(lang('en-US',SynsetId))),
  rdf_has(Synset, rdfs:label, literal(lang('en-US', Label))).

synset_from_id(Synset, SynsetID) :-
    rdf_has(Synset, wn20schema:synsetId, literal(lang('en-US', SynsetID))).

%
% Checks the rdf:type of the given synset.
%
synset_type(Synset, Type):-
     rdf_has(Synset, rdf:type, Type).

%
% Checks if the given word Label belongs to a synset given by SynsetUri.
%
word_has_synset_uri(Label, SynsetUri):- 
  rdf_has(SynsetUri, rdfs:label, literal(lang('en-US',Label))).

%
% Checks if the given word Label belongs to a synset given by SynsetUri that has the rdf type SynsetType.
%
word_has_synset_of_type(Label, SynsetUri, SynsetType):- 
  aggregate(count,
            and( word_has_synset_uri(Label, SynsetUri),
                 synset_type(SynsetUri, SynsetType) ), 3 ).

%
% Logical AND
% 
and(X,Y) :- X, Y.