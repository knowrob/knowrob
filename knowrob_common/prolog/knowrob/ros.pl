/*
  Copyright (C) 2019 Daniel Beßler
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
*/

:- module(knowrob_ros,
    [
      ros_type_path/2,
      ros_primitive_type/2
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/rdfs')).

:- rdf_meta ros_type_path(r,?),
            ros_primitive_type(?,r).

%% ros_type_path(+MessageType,?TypePath) is det.
%
ros_type_path(MessageType,TypePath) :-
  rdfs_individual_of(MessageType,ros:'MessageType'),
  rdf_has_prolog(MessageType,ros:hasTypePath,TypePath),!.
ros_type_path(PrimitiveType,TypePath) :-
  rdfs_individual_of(PrimitiveType,ros:'PrimitiveType'),
  rdf_split_url(_, TypePath, PrimitiveType),!.
ros_type_path(ArrayType, ArrayType_atom) :-
  rdfs_individual_of(ArrayType,ros:'ArrayType'),
  once((
    rdf_has(ArrayType,dul:hasPart,X),
    ros_type_path(X,T))),
  term_to_atom(array(T), ArrayType_atom),!.

%% ros_primitive_type(?ROS_type, ?RDF_type) is det.
%
% A mapping between ROS and RDF types.
%
ros_primitive_type('bool',    'http://www.w3.org/2001/XMLSchema#boolean').
ros_primitive_type('float32', 'http://www.w3.org/2001/XMLSchema#float').
ros_primitive_type('float64', 'http://www.w3.org/2001/XMLSchema#double').
ros_primitive_type('int8',    'http://www.w3.org/2001/XMLSchema#byte').
ros_primitive_type('int16',   'http://www.w3.org/2001/XMLSchema#short').
ros_primitive_type('int32',   'http://www.w3.org/2001/XMLSchema#int').
ros_primitive_type('int64',   'http://www.w3.org/2001/XMLSchema#long').
ros_primitive_type('uint8',   'http://www.w3.org/2001/XMLSchema#unsignedByte').
ros_primitive_type('uint16',  'http://www.w3.org/2001/XMLSchema#unsignedShort').
ros_primitive_type('uint32',  'http://www.w3.org/2001/XMLSchema#unsignedInt').
ros_primitive_type('uint64',  'http://www.w3.org/2001/XMLSchema#unsignedLong').
ros_primitive_type('string',  'http://www.w3.org/2001/XMLSchema#string').
% TODO support duration and time
%ros_primitive_type('duration',_).
%ros_primitive_type('time',xsd:dateTime).

ros_array_type('bool',    'http://knowrob.org/kb/knowrob.owl#array_boolean').
ros_array_type('float32', 'http://knowrob.org/kb/knowrob.owl#array_float').
ros_array_type('float64', 'http://knowrob.org/kb/knowrob.owl#array_double').
ros_array_type('int8',    'http://knowrob.org/kb/knowrob.owl#array_int').
ros_array_type('int16',   'http://knowrob.org/kb/knowrob.owl#array_int').
ros_array_type('int32',   'http://knowrob.org/kb/knowrob.owl#array_int').
ros_array_type('int64',   'http://knowrob.org/kb/knowrob.owl#array_int').
ros_array_type('uint8',   'http://knowrob.org/kb/knowrob.owl#array_uint').
ros_array_type('uint16',  'http://knowrob.org/kb/knowrob.owl#array_uint').
ros_array_type('uint32',  'http://knowrob.org/kb/knowrob.owl#array_uint').
ros_array_type('uint64',  'http://knowrob.org/kb/knowrob.owl#array_uint').
ros_array_type('string',  'http://knowrob.org/kb/knowrob.owl#array_string').
