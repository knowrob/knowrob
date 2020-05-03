:- module(model_notify, []).

:- use_module(library('semweb/rdf_db'),
    [ rdf_split_url/3
    ]).
:- use_module(library('db/tripledb'),
    [ tripledb_ask/3
    ]).
:- use_module(library('lang/terms/holds'),
    [ holds/3
    ]).
:- use_module(library('lang/terms/is_a'),
    [ subproperty_of/2
    ]).

:- use_module('./RDFS.pl'
    [ has_type/2,
      has_property_range/2
    ]).
:- use_module('./DUL/Event.pl'
    [ is_event/1,
      is_task/1,
      is_process_type/1,
      is_state_type/1
    ]).
:- use_module('./DUL/Object.pl'
    [ is_object/1,
      is_physical_object/1,
      is_physical_object/1
    ]).
:- use_module('./DUL/Region.pl'
    [ is_time_interval/1
    ]).

:- rdf_meta initialize_required_(r,r).

%%
%
%
notify_hook(individual(X)) :-
  ( is_object(X) -> notify(object(X)) ;
    is_event(X)  -> notify(event(X)) ;
    fail
  ).

notify_hook(object(Object)) :-
  initialize_required_(Object,dul:hasQuality),
  ( is_physical_object(Object) -> initialize_PO_(Object);
    is_concept(Object)         -> initialize_CO_(Object);
    true
  ).

notify_hook(event(Evt)) :-
  initialize_EV_(Evt).

%%
initialize_EV_(Evt) :-
  universal_scope(US),
  tell([ is_time_interval(TimeInterval),
         holds(Evt,dul:hasTimeInterval,TimeInterval)
       ],US).
  
%%
initialize_PO_(PO) :-
  initialize_lifetime_(Object),
  initialize_required_(Object,ease_obj:hasFeature),
  initialize_localization_(Object).
  
%%
initialize_lifetime_(Object) :-
  ( tripledb_ask(Obj,ease:hasLifetime,_) ; (
    universal_scope(US),
    tell([ is_event(Evt),
           holds(Obj,ease:hasLifetime,Evt)
         ],US)
  )),!.

%%
initialize_localization_(Object) :-
  % TODO this is ROS-specific and probably should not be done here, or?
  object_localization(Obj,Loc),
  ( has_region(Loc,_) -> true ; (
    % create a new region if none exist,
    % and assign a frame name to the region used
    % for pose data lookups in the DB.
    rdf_split_url(_,Frame,Object),
    universal_scope(US),
    tell([ instance_of(Region,knowrob:'Pose'),
           holds(Region,knowrob:hasFrameName,Frame),
           has_region(Loc,Region) ], US)
  )).
  
%%
initialize_CO_(C) :-
  initialize_required_(C,dul:hasParameter),
  ( is_task(C)         -> initialize_required_(C,dul:isTaskOf);
    is_process_type(C) -> initialize_required_(C,ease_proc:isProcessTypeOf);
    is_state_type(C)   -> initialize_required_(C,ease_proc:isStateTypeOf);
    true
  ).

%%
%
initialize_required_(Object,P) :-
  findall([R0,Min0],
    % NOTE: this also yields general statements
    holds(Object,P,min(Min0,R0)),
    MinR),
  member([ValueType,Min],MinR),
  % HACK: skip if there are is an axiom for a sub-class of R.
  %        might be fine for most situations, but can create issues!
  % TODO: consider axioms on subclasses of R to reduce number of
  %        created entities
  \+ ( member(R2,_), subclass_of(R2,ValueType) ),
  % get current cardinality
  aggregate_all(count, (
    tripledb_ask(Object,P,Val),
    has_type(Val,ValueType)
  ), NumActual),
  NumMissing is Min - NumActual,
  % TODO: infer more specific P. This might not be trivial...
  %          - could we get this from calling holds above?
  %          - else probably need to do some more queries
  % HACK: for now try to use sub-properties of P with matching min card
  get_property_(Object,P,ValueType,Min,P_sup),
  % finally generate symbols, and assert relation
  forall(
    between(1,NumMissing,_),
    tell(
      instance_of(X,ValueType),
      holds(Object,P_sup,X)
    )
  ).

%%
get_property_(Object,P,Range,Min,P_Sub) :-
  has_property_range(P_Sub,Range),
  subproperty_of(P_Sub,P),
  holds(Object,P_Sub,min(Min,Range)),
  !.
get_property_(_,P,_,_,P).
