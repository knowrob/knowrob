/*
  Copyright (C) 2015 Daniel Beßler
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

:- module(mongo_images,
    [
        mng_kinect_head_rgb_image_color/2,
        mng_image_base64/2,
        mng_image_base64_compresssed/2
    ]).
/** <module> Looking up images in a mongo DB

@author Daniel Beßler
@license BSD
*/

:- use_module(library('jpl')).
:- use_module(library('knowrob/mongo')).

:-  rdf_meta
    mng_kinect_head_rgb_image_color(+,-),
    mng_image_base64(+,-),
    mng_image_base64_compresssed(+,-).

%% mng_kinect_head_rgb_image_color(+Instant, -DBObj) is semidet
%
% Query for latest image message published on kinect_head_rgb_image_color topic before Instant
% and unify DBObj with the Java object that holds the image information.
%
mng_kinect_head_rgb_image_color(Instant, DBObj) :-
  mng_query_latest('kinect_head_rgb_image_color', one(DBObj), 'header.stamp', Instant).

%% mng_image_base64(+DBObj, -Base64) is semidet
%
% Unifies Base64 with the base64 encoded atom that corresponds
% to the pixels of the uncomporessed image DB record DBObj.
% DBObject is a recorded message of type sensor_msgs/Image.
%
mng_image_base64(DBObj, Base64) :-
  mng_republisher(_),
  jpl_call('org.knowrob.utils.ros.MongoImageEncoding', 'encodeBase64', [DBObj], Base64).

%% mng_image_base64_compresssed(+DBObj, -Base64) is semidet
%
% Unifies Base64 with the base64 encoded atom that corresponds
% to the pixels of the comporessed image DB record DBObj.
% DBObject is a recorded message of type sensor_msgs/CompressedImage.
%
mng_image_base64_compresssed(DBObj, Base64) :-
  mng_republisher(_),
  jpl_call('org.knowrob.utils.ros.MongoImageEncoding', 'encodeBase64_compressed', [DBObj], Base64).
