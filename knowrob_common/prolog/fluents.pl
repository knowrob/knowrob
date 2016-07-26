/** <module> Reusable fluent ontology

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

:- module(fluents,
    [
      assert_fluent_begin/3,
      assert_fluent_end/3,
      assert_fluent_end/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).

:- rdf_meta assert_fluent_begin(r,r,r),
            assert_fluent_end(r,r,r),
            assert_fluent_end(r,r).

assert_fluent_begin(Subject, Predicate, Object) :-
  true.

assert_fluent_end(Subject, Predicate) :-
  true.

assert_fluent_end(Subject, Predicate, Object) :-
  true.