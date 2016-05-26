/** <module> Core part of the KnowRob language

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

:- module(knowrob_language,
    [
      holds/1,
      holds/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).

:- rdf_meta holds(t),
            holds(t,?).

%% holds(+Term, ?T)
%% holds(+Term)
%
% True iff @Term holds during @T.
% 
% @T can be TimeInterval or TimePoint individual, a number or a list of two numbers
% representing an time interval.
%
% @param Term Must be of the form: "PROPERTY(SUBJECT, OBJECT)".
%             For example: `Term = knowrob:insideOf(example:'DinnerPlate_fdigh245', example:'Drawer_bsdgwe8trg')`.
% @param T Can be TimeInterval or TimePoint individual, a number or a list of two numbers
%          representing a time interval.
%
holds(Term) :-
  current_time(T),
  holds(Term,T).

holds(Term, T) :-
  (  Term =.. [':', Namespace, Tail]
  -> (
     Tail =.. [Property,Subject,Object],
     % unpack namespace
     rdf_current_ns(Namespace, NamespaceUri),
     atom_concat(NamespaceUri, Property, PropertyUri),
     holds(PropertyUri, Subject, Object, T)
  ) ; (
     Term =.. [Property,Subject,Object],
     holds(Property, Subject, Object, T)
  )).

holds(Property, Subject, Object, T) :-
  rdf_triple(Property, Subject, Object),
  not( rdfs_individual_of(Subject, knowrob:'TemporalPart') ),
  (  var(T)
  -> T = [0.0,inf]
  ;  true
  ), !.
  
holds(Property, Subject, Object, T) :-
  rdfs_individual_of(Property, owl:'DatatypeProperty'),
  % query temporal part that declares '@Property = @Object'
  rdf_has(SubjectPart, knowrob:'temporalPartOf', Subject),
  once(rdf_triple(Property, SubjectPart, Object)),
  % match time interval with @T
  rdf_has(SubjectPart, knowrob:'temporalExtend', TimeInterval),
  time_term(TimeInterval, [T0,T1]),
  (  var(T)
  -> T = [T0,T1]
  ;  time_between(T, T0, T1)
  ).
  
holds(Property, Subject, Object, T) :-
  rdfs_individual_of(Property, owl:'ObjectProperty'),
  % query temporal part that declares '@Property = ObjectPart'
  % where ObjectPart is a temporal part of @Object
  rdf_has(SubjectPart, knowrob:'temporalPartOf', Subject),
  once(rdf_triple(Property, SubjectPart, ObjectPart)),
  rdf_has(ObjectPart, knowrob:'temporalPartOf', Object),
  % match time interval with @T
  rdf_has(SubjectPart, knowrob:'temporalExtend', TimeInterval),
  time_term(TimeInterval, [T0,T1]),
  (  var(T)
  -> T = [T0,T1]
  ;  time_between(T, T0, T1)
  ).
