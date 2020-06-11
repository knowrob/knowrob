:- module(model_notify, []).

:- use_module(library('semweb/rdf_db'),
    [ rdf_split_url/3 ]).
:- use_module(library('db/tripledb'),
    [ tripledb_ask/3 ]).
:- use_module(library('db/scope'),
    [ universal_scope/1 ]).
:- use_module(library('lang/terms/holds'),
    [ holds/3 ]).
:- use_module(library('lang/terms/is_a'),
    [ subproperty_of/2 ]).

:- use_module('./RDFS.pl',
    [ has_type/2,
      has_range/2
    ]).
:- use_module('./DUL/Event.pl',
    [ is_event/1,
      is_task/1
    ]).
:- use_module('./DUL/Object.pl',
    [ is_object/1,
      is_concept/1,
      is_physical_object/1,
      is_physical_object/1
    ]).
:- use_module('./DUL/Region.pl',
    [ is_time_interval/1 ]).
:- use_module('./EASE/PROC.pl',
    [ is_process_type/1 ]).
:- use_module('./EASE/STATE.pl',
    [ is_state_type/1 ]).

% FIXME: Syntax error: Operator expected
%:- rdf_meta initialize_required_(r,r).
:- rdf_meta(initialize_required_(r,r)).

%%
%
%
notify:notify_hook(individual(X)) :-
  ( is_object(X) -> notify(object(X)) ;
    is_event(X)  -> notify(event(X)) ;
    fail
  ).

notify:notify_hook(object(Object)) :-
  initialize_required_(Object,dul:hasQuality),
  ( is_physical_object(Object) -> initialize_PO_(Object);
    is_concept(Object)         -> initialize_CO_(Object);
    true
  ).

notify:notify_hook(event(Evt)) :-
  initialize_EV_(Evt).

%%
initialize_EV_(EV) :-
  \+ holds(EV,dul:hasTimeInterval,_),
  tell([ is_time_interval(TI),
         holds(EV,dul:hasTimeInterval,TI) ]).
  
%%
initialize_PO_(PO) :-
  initialize_lifetime_(PO),
  initialize_required_(PO,ease_obj:hasFeature),
  initialize_LOC_(PO).
  
%%
initialize_lifetime_(O) :-
  tripledb_ask(O,knowrob:hasLifetime,_),!.

initialize_lifetime_(O) :-
  tell([ is_event(EV),
         holds(O,knowrob:hasLifetime,EV) ]).

%%
initialize_LOC_(PO) :-
  % TODO: better would be an axiom for all POs, then this clause would be obsolete
  ( object_localization(PO,LOC) -> true ; (
    tell([ instance_of(LOC,ease_obj:'Localization'),
           holds(PO,ease_obj:hasLocalization,LOC) ])
  )),
  %%
  ( has_region(LOC,_) -> true ; (
    % create a new region if none exist,
    % and assign a frame name to the region used
    % for pose data lookups in the DB.
    rdf_split_url(_,Frame,PO),
    tell([ instance_of(Region,knowrob:'Pose'),
           holds(Region,knowrob:frameName,Frame),
           has_region(LOC,Region) ])
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
  findall([Range0,Card0],
    % NOTE: this also yields general statements
    holds(Object,P,min(Card0,Range0)),
    Required),
  list_to_set(Required,Set),
  forall(
    member([Range1,Card1],Set), (
    reduced_cardinality_(Range1,Set,Card1->Card2),
    once((
      Card2=<0;
      initialize_required_1_(Object,P,[Range1,Card2])
    ))
  )).

%%
% FIXME
reduced_cardinality_(Range0,List,_->0) :-
  member([Range1,_],List),
  Range1\=Range0,
  subclass_of(Range1,Range0),!.
reduced_cardinality_(_,_,N->N).

%%
initialize_required_1_(Object,P,[Range,Card]) :-
  % get current cardinality
  aggregate_all(count,
    ( tripledb_ask(Object,P,Val),
      has_type(Val,Range) ),
    NumActual
  ),
  NumMissing is Card - NumActual,
  % TODO: infer more specific P. This might not be trivial...
  %          - could we get this from calling holds above?
  %          - else probably need to do some more queries
  % HACK: for now try to use sub-properties of P with matching min card
  get_property_(Object,P,Range,Card,P_sup),
  print_message(informational,
      notify(initialize(Object,P_sup,NumMissing,Range))),
  % finally generate symbols, and assert relation
  forall(
    between(1,NumMissing,_),
    tell([ instance_of(X,Range),
           holds(Object,P_sup,X) ])
  ).

%%
get_property_(Object,P,Range,Min,P_Sub) :-
  has_range(P_Sub,Range),
  subproperty_of(P_Sub,P),
  holds(Object,P_Sub,min(Min,Range)),
  !.
get_property_(_,P,_,_,P).
