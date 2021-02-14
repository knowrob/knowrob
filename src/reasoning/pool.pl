:- module(reasoning_pool,
    [ register_reasoner/1,
      infer/3
    ]).
/** <module> TODO ...

@author Daniel Beßler
*/

:- use_module(library('lang/scope'),
    [ scope_intersect/3, time_scope_data/2 ]).

:- dynamic reasoner_module_/1.
:- dynamic active_query_/2.

%% rendering of messages
prolog:message(reasoner(Module,inferred(Fact))) -->
	[ 'A reasoner (~w) has inferred new knowledge (~w)'-[Module,Fact] ].

%% register_reasoner(+Module) is det.
%
%
register_reasoner(Module) :-
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
  % select registered reasoner that did not yet answer 
  % given query, but is capable to do so
  reasoner_module_(Module),
  can_answer_(Module,Query),
  \+ active_query_(Module,QueryStr),
  % finally perform the inference
  setup_call_cleanup(
      % remember that the reasoner has answered this query
      asserta(active_query_(Module,QueryStr)),
      % infer solutions
      infer1(Module,Query,Fact,QScope,FScope0),
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

infer1(Module,Query,Fact,QScope,FScope) :-
  findall([X,Y], (
    % FIXME: do not invalidate cache for self?
    call((:(Module,infer(Query,X,QScope,Y)))),
    ( lang_query:tell(X,Y,[]) -> true ; (
      print_message(error, infer(failed(tell(X,Y))))
    ))
  ), Results),
  member([Fact,FScope],Results),
  log_debug(reasoner(Module,inferred(Fact))),
  % FIXME
  Query=Fact.

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

