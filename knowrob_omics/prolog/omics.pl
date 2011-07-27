/** <module> omics

This module contains predicates derived from the omics db

Copyright (c) 2011, Lars Kunze <kunzel@cs.tum.edu>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Intelligent Autonomous Systems Group/
      Technische Universitaet Muenchen nor the names of its contributors 
      may be used to endorse or promote products derived from this software 
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

@author Lars Kunze
@license BSD
*/

:- module(omics,
          [
           probability_given/4,
           bayes_probability_given/4
           ]).


% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:-  rdf_meta
  num_of_entries(r,-),
  num_of_subjects(r,r,-),
  num_of_objects(r,r,-),
  num_of_distinct_subjects(r,-),
  num_of_distinct_objects(r,-),
  num_of_entries_per_object(r,r,r,-),
  probability_subj(r,r,-),
  probability_obj(r,r,-),
  probability_given(r,r,r,-),
  bayes_probability_given(r,r,r,-).

  
% Lambda is < 1, typical value 0.5, laplace =1
% influence how unseen objects are  
lambda(Lambda):-
  Lambda is 0.5.

num_of_entries(Type, N):-
  findall(E, (rdf_has(E,rdf:type,Type)), Es), length(Es,N).

num_of_subjects(Type, Subj, N):-
  findall(Subj, (rdf_has(E,rdf:type,Type),
                 rdf_has(E, knowrob:subject, Subj)),
          Ss),
  length(Ss,N).

num_of_distinct_subjects(Type, N):-
  findall(Subj, (rdf_has(E,rdf:type,Type),
                 rdf_has(E, knowrob:subject,Subj)),
          Ss),
  list_to_set(Ss, Set),
  length(Set, N).

num_of_distinct_objects(Type, N):-
  findall(Obj, (rdf_has(E,rdf:type,Type),
                rdf_has(E, knowrob:object,Obj)),
          Os),
  list_to_set(Os, Set),
  length(Set, N).

num_of_objects(Type, Obj, N):-
  findall(Obj, (rdf_has(E,rdf:type,Type),
                rdf_has(E, knowrob:object, Obj)),
          Os),
  length(Os,N).

num_of_entries_per_object(Type, S, O, N):-
  findall(Subj, (rdf_has(E,rdf:type,Type),
                 rdf_has(E, knowrob:subject, Subj)),
          Ss),
  list_to_set(Ss,SSet),
  findall(Obj, (rdf_has(E,rdf:type,Type),
                rdf_has(E, knowrob:object, Obj)),
          Os),
  list_to_set(Os,OSet),
  member(S,SSet),
  member(O,OSet),
  findall([S,O], (rdf_has(E,rdf:type,Type),
                  rdf_has(E, knowrob:subject, S),
                  rdf_has(E, knowrob:object,  O)),
          List),
  length(List,N).

probability_subj(Type,S,P):-
  findall(Subj, (rdf_has(E,rdf:type,Type),
                 rdf_has(E, knowrob:subject, Subj)),
          Ss),
  list_to_set(Ss,SSet),
  member(S, SSet),
  num_of_subjects(Type,S,N1),
  num_of_entries(Type, N2),
  P is N1 / N2.

probability_obj(Type,O,P):-
  findall(Obj, (rdf_has(E,rdf:type,Type),
                rdf_has(E, knowrob:object, Obj)),
          Os),
  list_to_set(Os,OSet),
  member(O, OSet),
  num_of_objects(Type,O,N1),
  num_of_entries(Type, N2),
  P is N1 / N2.

probability_given(Type, Subj, Obj, P):-
  lambda(L),
  num_of_entries_per_object(Type, Subj, Obj, N1),
  num_of_objects(Type, Obj, N2),
  num_of_distinct_subjects(Type, N3),
  P is (N1  + L) /  (N2 + (L * N3)). % Use Lid-stone’s law of succession, to assign probability to unseen objects 

bayes_probability_given(Type, Obj, Subj, P):-
  % calc numerator
  probability_given(Type, Subj, Obj, P1),
  probability_obj(Type,Obj,P2),

  % calc denominator
  findall(P3, (probability_given(Type, Subj, O, P4),
               probability_obj(Type, Obj,P5),
               P3 is P4 * P5),
          Ps),
  sumlist(Ps, P6),
  P is (P1 * P2) / P6. 
  


% locations db
% :-rdf_assert(entry1, rdf:type,knowrob:'OmicsLocations',omics).
% :-rdf_assert(entry1,'http://ias.cs.tum.edu/kb/knowrob.owl#subject','http://ias.cs.tum.edu/kb/knowrob.owl#Cup',omics).
% :-rdf_assert(entry1,'http://ias.cs.tum.edu/kb/knowrob.owl#object','http://ias.cs.tum.edu/kb/knowrob.owl#Kitchen',omics).

% :-rdf_assert(entry2,rdf:type,'http://ias.cs.tum.edu/kb/knowrob.owl#OmicsLocations',omics).
% :-rdf_assert(entry2,'http://ias.cs.tum.edu/kb/knowrob.owl#subject','http://ias.cs.tum.edu/kb/knowrob.owl#Cup',omics).
% :-rdf_assert(entry2,'http://ias.cs.tum.edu/kb/knowrob.owl#object','http://ias.cs.tum.edu/kb/knowrob.owl#Kitchen',omics).

% :-rdf_assert(entry3,rdf:type,'http://ias.cs.tum.edu/kb/knowrob.owl#OmicsLocations',omics).
% :-rdf_assert(entry3,'http://ias.cs.tum.edu/kb/knowrob.owl#subject','http://ias.cs.tum.edu/kb/knowrob.owl#Sandwich',omics).
% :-rdf_assert(entry3,'http://ias.cs.tum.edu/kb/knowrob.owl#object','http://ias.cs.tum.edu/kb/knowrob.owl#Kitchen',omics).

% :-rdf_assert(entry4,rdf:type,'http://ias.cs.tum.edu/kb/knowrob.owl#OmicsLocations',omics).
% :-rdf_assert(entry4,'http://ias.cs.tum.edu/kb/knowrob.owl#subject','http://ias.cs.tum.edu/kb/knowrob.owl#Sandwich',omics).
% :-rdf_assert(entry4,'http://ias.cs.tum.edu/kb/knowrob.owl#object','http://ias.cs.tum.edu/kb/knowrob.owl#Restaurant',omics).

% :-rdf_assert(entry5,rdf:type,'http://ias.cs.tum.edu/kb/knowrob.owl#OmicsLocations',omics).
% :-rdf_assert(entry5,'http://ias.cs.tum.edu/kb/knowrob.owl#subject','http://ias.cs.tum.edu/kb/knowrob.owl#Sandwich',omics).
% :-rdf_assert(entry5,'http://ias.cs.tum.edu/kb/knowrob.owl#object','http://ias.cs.tum.edu/kb/knowrob.owl#Restaurant',omics).

% :-rdf_assert(entry6,rdf:type,'http://ias.cs.tum.edu/kb/knowrob.owl#OmicsLocations',omics).
% :-rdf_assert(entry6,'http://ias.cs.tum.edu/kb/knowrob.owl#subject','http://ias.cs.tum.edu/kb/knowrob.owl#Plate',omics).
% :-rdf_assert(entry6,'http://ias.cs.tum.edu/kb/knowrob.owl#object','http://ias.cs.tum.edu/kb/knowrob.owl#Restaurant',omics).

% :-rdf_assert(entry7,rdf:type,'http://ias.cs.tum.edu/kb/knowrob.owl#OmicsLocations',omics).
% :-rdf_assert(entry7,'http://ias.cs.tum.edu/kb/knowrob.owl#subject','http://ias.cs.tum.edu/kb/knowrob.owl#Saucer',omics).
% :-rdf_assert(entry7,'http://ias.cs.tum.edu/kb/knowrob.owl#object','http://ias.cs.tum.edu/kb/knowrob.owl#DiningRoom',omics).




