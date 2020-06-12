:- module(reasoning_pool,
    [ reasoning_module/1,
      infer(t,-,t)
    ]).
/** <module> TODO ...

@author Daniel Beßler
*/

:- use_module(library('db/tripledb'),
    [ tripledb_cache_get/3,
      tripledb_cache_add/3
    ]).
:- use_module(library('db/scope'),
    [ scope_intersect/3 ]).
:- use_module(library('lang/scopes/temporal'),
    [ time_scope_data/2 ]).

:- dynamic reasoner_module_/1.
:- dynamic active_query_/2.

%% reasoning_module(+Module) is det.
%
%
reasoning_module(Module) :-
  assertz( reasoner_module_(Module) ).

%%
can_answer_(Module,Query) :-
  % avoid binding vars in Query
  copy_term(Query, Query0),
  call((:(Module,can_answer(Query0)))),!.

can_answer__(instance_of(A,B)    , instance_of(A,B)).
can_answer__(subclass_of(A,B)    , subclass_of(A,B)).
can_answer__(subproperty_of(A,B) , subproperty_of(A,B)).
can_answer__(holds_object(S,P,O) , holds(S,P,O)).
can_answer__(holds_data(S,P,O)   , holds(S,P,O)).

%%
%
%
%
infer(Query0,Fact,QScope->FScope) :-
  once(can_answer__(Query0,Query)),
  query_string_(Query,QueryStr),
  % get list of reasoner that already answered this
  % query.
  ( is_cachable_(Query,Pred) ->
    tripledb_cache_get(Pred,QueryStr,CachedModules);
    CachedModules=[]
  ),
  % select registered reasoner that did not yet answer 
  % given query, but is capable to do so
  reasoner_module_(Module),
  can_answer_(Module,Query),
  \+ memberchk(Module,CachedModules),
  \+ active_query_(Module,QueryStr),
  % finally perform the inference
  setup_call_cleanup(
      % remember that the reasoner has answered this query
      asserta(active_query_(Module,QueryStr)),
      % infer solutions
      infer1(Module,Query,QueryStr,Fact,QScope,FScope0),
      % indicate that we are done with end_of_inference atom
      retractall(active_query_(Module,QueryStr))
  ),
  % force until=now in FScope if var(Until)
  time_scope_data(FScope0,[_,Until]),
  ( ground(Until) -> FScope=FScope0 ; (
    get_time(Now),
    time_scope_get(0,_,Now,_,UntilNowScope),
    scope_intersect(FScope0,UntilNowScope,FScope)
  )).

infer1(Module,Query,QueryStr,Fact,QScope,FScope) :-
  add_to_cache_(Module,Query,QueryStr),
  Options=[skip_invalidate(true)],
  % FIXME: better use thread queues to yield a resut as soon as it was inferred
  findall([X,Y], (
    call((:(Module,infer(Query,X,QScope,Y)))),
    % FIXME: do not invalidate cache for self?
    ( tell(X,[Options,Y]) -> true ; (
      print_message(error, infer(failed(tell(X,Y))))
    ))
  ), Results),
  member([Fact,FScope],Results),
  print_message(informational, inferred(Module,Fact)),
  % FIXME
  Query=Fact.

% TODO: run reasoner in a thread.
%        - but reasoner calls itself recursively, creating too many threads!
%infer1(Module,Query,QueryStr,Fact,QScope,FScope) :-
  %% run the reasoner in a separate thread where it
  %% fills a queue with inferred answers.
  %message_queue_create(Queue),
  %thread_create(
    %infer_thread_(Module,Query,QueryStr,QScope,Queue),
    %_ThreadId),
  %% FIXME: message_queue_destroy never called in case not all results
  %%          are pulled
  %poll_result(Queue,Fact,FScope),
  %Query=Fact.

%infer_thread_(Module,Query,QueryStr,QScope,Queue) :-
  %setup_call_cleanup(
      %% remember that the reasoner has answered this query
      %add_to_cache_(Module,Query,QueryStr),
      %% infer solutions
      %catch(infer_thread_1_(Module,Query,QScope,Queue),
            %Exception,
            %print_message(error,reasoner(exception(Module,Exception)))),
      %% indicate that we are done with end_of_inference atom
      %thread_send_message(Queue,end_of_inference)
  %).

%%
%infer_thread_1_(Module,Query,QScope,Queue) :-
  %forall(
    %% call the reasoner
    %call((:(Module,infer(Query,Fact,QScope,FScope)))),
    %% handle inference
    %( thread_send_message(Queue,[Fact,FScope]),
      %% assert inferred triple to DB
      %tell(Fact,FScope) )
  %).

%%
add_to_cache_(Module,Query,QueryStr) :-
  ( is_cachable_(Query,Pred) ->
    tripledb_cache_add(Pred,QueryStr,Module);
    true ).

%%
poll_result(Queue,Fact,Scope) :-
  thread_get_message(Queue,Msg),
  ( Msg=end_of_inference ->
    ( message_queue_destroy(Queue), fail );
    ( Msg=[Fact,Scope] ; poll_result(Queue,Fact,Scope) )
  ).

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
    query_string1_(L0,Y0)
  ), YL),
  Y=..YL.

query_string1_(X,X).

%%
%
is_cachable_(subclass_of(_,_),    subclass_of).
is_cachable_(subproperty_of(_,_), subproperty_of).
