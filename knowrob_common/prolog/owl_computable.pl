/*  $Id$

    Extensions to merge OWL reasoning with computables.

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

:- module(owl_computable,
	  [ 
	    owl_computable_db/1,
	    owl_instance_of/2,
	    owl_triple/3			% ?Subject, ?Predicate, ?Object
	  ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('rdfs_computable')).
:- use_module(library('owl')).

:- rdf_meta
  owl_computable_db(-),
  owl_instance_of(r, t),
  owl_triple(r, r, o).

%% owl_computable_db
owl_computable_db(db(rdf_triple,rdfs_instance_of)).

     /*******************************
     *     INDIVIDUAL OF  *
     *******************************/

%%  owl_instance_of(?Resource, +Description) is nondet.
%
% Test  or  generate  the  resources    that  satisfy  Description
% according the the OWL-Description entailment rules.

owl_instance_of(Resource, Description) :-
  owl_computable_db(DB),
  owl_individual_of(Resource, Description, DB).

		 /*******************************
		 *	  OWL PROPERTIES	*
		 *******************************/

%%	owl_has(?Subject, ?Predicate, ?Object)
%
%	True if this relation is specified or can be deduced using OWL
%	inference rules.  It adds transitivity to owl_has_direct/3.

owl_triple(S, P, O) :-
  owl_computable_db(DB),
  owl_has(S, P, O, DB).

