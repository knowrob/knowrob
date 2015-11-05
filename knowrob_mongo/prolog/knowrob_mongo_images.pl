/** 

  Copyright (C) 2013 Moritz Tenorth, 2015 Daniel Beßler
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

@author Moritz Tenorth
@author Daniel Beßler
@license BSD
*/

:- module(knowrob_mongo_images,
    [
        mng_kinect_frame_before/2,
        mng_image_before/2,
        mng_image_base64/2
    ]).

:- use_module(library('jpl')).
:- use_module(library('knowrob_mongo')).
:- use_module(library('knowrob_mongo_interface')).

:-  rdf_meta
    mng_kinect_frame_before(r, ?),
    mng_image_before(r, -),
    mng_image_base64(+,-).

mng_kinect_frame_before(Time, DBObj) :-
  mng_latest('kinect_head_rgb_image_color', DBObj, 'header.stamp', Time).

mng_image_before(Timepoint, DesigJava) :-
  atom(Timepoint),
  time_term(Timepoint, Time),
  mng_image_before(Time, DesigJava).

mng_image_before(Time, DesigJava) :-
  number(Time),
  mongo_interface(DB),
  jpl_call(DB, 'getLatestImageBefore', [Time], DesigJava),
  not(DesigJava = @(null)).

mng_image_base64(DBObj, Base64) :-
  jpl_call('org.knowrob.vis.ImageEncoding', 'encodeBase64', [DBObj], Base64).

%  mng_designator_props('IMAGE', Desig, 'HEIGHT', H),
%  mng_designator_props('IMAGE', Desig, 'WIDTH', W),
%  mng_designator_props('IMAGE', Desig, 'ENCODING', Enc),
%  mng_designator_props('IMAGE', Desig, 'IS_BIGENDIAN', BigEndian),
%  mng_designator_props('IMAGE', Desig, 'STEP', Step),
%  mng_designator_props('IMAGE', Desig, 'DATA', Data).

%mng_latest_image(Timepoint, DesigJava) :-
%  atom(Timepoint),
%  time_term(Timepoint, Time),
%  mng_latest_image(Time, DesigJava).

%mng_latest_image(Time, DesigJava) :-
%  number(Time),
%  mongo_interface(DB),
%  jpl_call(DB, 'getLatestImageBefore', [Time], DesigJava),
%  not(DesigJava = @(null)).
