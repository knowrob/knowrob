:- module(reasoning_pool,
    [ reasoning_module/1,
      infer/3
    ]).
/** <module> TODO ...

@author Daniel Beßler
*/

:- use_module(library('db/tripledb'),
    [ tripledb_cache_get/2,
      tripledb_cache_add/3
    ]).
:- use_module(library('lang/query'),
    [ scope_intersect/3
    ]).
:- use_module(library('lang/scopes/temporal'),
    [ time_scope_data/2
    ]).

:- dynamic reasoner_module_/1.

%% reasoning_module(+Module) is det.
%
%
reasoning_module(Module) :-
  % TODO: maybe better to get the module predicates
  %         here? does it make a difference?
  assertz( reasoner_module_(Module) ).

%%
can_answer_(Module,Query,Meta) :-
  call((:(Module,can_answer(Query,Meta)))).

%%
%
%
%
infer(Query,Fact,QScope->FScope) :-
  query_string_(Query,QueryStr),
  % get list of reasoner that already answered this
  % query.
  tripledb_cache_get(QueryStr,L),
  % select registered reasoner that did not yet answer 
  % given query, but is capable to do so
  reasoner_module_(Module),
  can_answer_(Module,Query),
  \+ member(Module,L),
  % finally perform the inference
  % TODO: run reasoner in parallel
  infer1(Module,Query,QueryStr,Fact,QScope,FScope0),
  % force until=now in FScope if var(Until)
  time_scope_data(FScope0,[_,Until]),
  ( ground(Until) -> FScope=FScope0 ; (
    get_time(Now),
    time_scope_get(0,_,Now,_,UntilNowScope),
    scope_intersect(FScope0,UntilNowScope,FScope)
  )).

infer1(Module,Query,QueryStr,Fact,QScope,FScope) :-
  % run the reasoner in a separate thread where it
  % fills a queue with inferred answers.
  message_queue_create(Queue),
  thread_create(
    infer_thread_(Module,Query,QueryStr,QScope,Queue),
    _ThreadId),
  % poll reasoning results until end_of_inference
  % FIXME: message_queue_destroy never called in case not all results
  %          are pulled
  poll_result(Queue,Fact,FScope).

infer_thread_(Module,Query,QueryStr,QScope,Queue) :-
  % remember that the reasoner has answered this query
  ( \+ is_cachable_(Query,Pred) ;
    tripledb_cache_add(Module,Pred,QueryStr)
  ),!,
  % infer solutions
  forall(
    % call the reasoner
    call((:(Module,infer(Query,Fact,QScope,FScope)))),
    % handle inference
    ( thread_send_message(Queue,[Fact,FScope]),
      % assert inferred triple to DB
      tell(Fact,FScope)
    )
  ),
  % indicate that we are done with end_of_inference atom
  thread_send_message(Queue,end_of_inference).

%%
poll_result(Queue,Fact,Scope) :-
  thread_get_message(Queue,Msg),
  ( Msg=end_of_inference ->
    message_queue_destroy(Queue); (
    Msg=[Fact,Scope] ;
    poll_result(Queue,Fact,Scope)
  )).

%%
% Unique query string used to identify cached
% queries.
%
query_string_(X,Y) :-
  query_string1_(X,X1),
  term_to_atom(X1,Y).

query_string1_(X,'_') :-
  var(X),!.

query_string1_(X,Y) :-
  compound(X),!,
  X=..L,
  findall(Y0, (
    member(L0,L),
    query_string_(L0,Y0)
  ), YL),
  Y=..YL.

query_string1_(X,X).

%%
%
is_cachable_(subclass_of(_,_),    subclass_of).
is_cachable_(subproperty_of(_,_), subproperty_of).
