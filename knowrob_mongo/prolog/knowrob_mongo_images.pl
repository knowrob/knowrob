
:- module(knowrob_mongo_images,
    [
        mng_kinect_frame_before/2,
        mng_image_before/2
    ]).

:- use_module(library('jpl')).
:- use_module(library('knowrob_mongo')).
:- use_module(library('knowrob_mongo_interface')).

:-  rdf_meta
    mng_kinect_frame_before(r, ?),
    mng_image_before(r, -).

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