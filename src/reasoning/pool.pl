
:- module(reasoning_pool,
    [ infer/3
    ]).

:- dynamic registered_reasoner/1.

% TODO:
%  - extend tripledb interface:
%       tripledb_ask_inferences, tripledb_add_inference
% TODO:
%  - run N reasoner in parallel
% FIXME:
% - how to handle when the triple db changes?
%     -- on tell reasoner could find new results
%     -- can tripledb handle it
%

%%
%
%
infer(Query,Fact,Scope) :-
  pending_inference(Query,Meta,Comm),!,
  % TODO
  % FIXME: need to fail when called by itself!
  fail.

infer(Query,Fact,QScope->FScope) :-
  % get list of reasoner that already answered this
  % query.
  tripledb_ask_inferences(Query,QScope,L),
  % select registered reasoner that did not yet answer 
  % given query, but is capable to do so
  registered_reasoner(Module),
  once(call(
    (:(Module,can_answer(Query)))
  )),
  \+ member(Module,L),
  % finally perform the inference
  infer1(Module,Query,Fact,QScope,FScope).

infer1(Module,Query,Fact,QScope,FScope) :-
  % run the reasoner in a separate thread where it
  % fills a queue with inferred answers.
  message_queue_create(Queue),
  thread_create(
    infer_thread_(Module,Query,QScope,Queue),
    _ThreadId),
  % FIXME: call
  % message_queue_destroy(Queue)
  % poll reasoning results until end_of_inference
  poll_result(Queue,Fact,FScope).

infer_thread_(Module,Query,QScope,Queue) :-
  % infer solutions
  forall(
    % call the reasoner
    call((:(Module,infer(Query,Fact,QScope,FScope)))),
    % handle inference
    ( thread_send_message(Queue,[Fact,FScope]),
      tell(Fact,FScope)
    )
  ),
  % indicate that we are done with end_of_inference atom
  thread_send_message(Queue,end_of_inference),
  % remember that the reasoner has answered this query
  % TODO: better assert in the beginning, and handle
  %         case that someone else asks the same during processing
  tripledb_add_inference(Query,QScope,Module).

%%
poll_result(Queue,Fact,Scope) :-
  thread_get_message(Queue,Msg),
  ( Msg=end_of_inference -> fail ; (
    Msg=[Fact,Scope] ;
    poll_result(Queue,Fact,Scope)
  )).

%%
reasoner_add(Module) :-
  assertz( registered_reasoner(Module) ).

%%
reasoner_can_answer(Module,Query,Meta) :-
  call((:(Module,can_answer(Query,Meta)))).
