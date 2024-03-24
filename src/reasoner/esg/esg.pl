:- module(esg, [
     esg/4,
     esg_truncated/4,
     esg_event_sequence/2,
     esg_assert/3,
     esg_retract/1,
     esg_to_list/2,
     esg_write_info/1,
     esg_write/1,
     esg_pop/3,
     esg_peak/2,
     esg_join/3,
     esg_endpoints/2,
     esg_path/4,
     endpoint_type/2
]).
/** <module> Event Endpoint Sequence Graph (ESG).

ESGs are directed acyclic graphs in which nodes are endpoints of constituent
events, and in which an edge is added from one endpoint a
to another endpoint b if a < b holds. Any path from endpoint
a to another endpoint b implies that a < b (transitivity).

@author Daniel Beßler
*/

:- dynamic esg_endpoint/3,       % sequencer id, endpoint id, endpoint term
           esg_endpoint_node/3,  % sequencer id, endpoint id, node id
           esg_edge/3.           % sequencer id, from endpoint id, to endpoint id

throw_unknown_endpoint(Endpoint) :-
  throw(model_error('Not a constituent',Endpoint)).
throw_axiom_contradiction(Axiom) :-
  throw(model_error('Contradictory axiom',Axiom)).

append_dl(Xs-Ys,Ys-Zs,Xs-Zs).

random_id(Id) :-
  randseq(8, 25, Seq_random),
  maplist(plus(65), Seq_random, Alpha_random),
  atom_codes(Id, Alpha_random).

%%%%%%%%%%%%%%

esg_unique_id(ESG) :-
  random_id(S),
  ( esg_endpoint(S,_,_) ->
    esg_unique_id(ESG) ;
    ESG = S ).

next_endpoint_id(ESG,E) :-
  random_id(E0),
  ( esg_endpoint(ESG,E0,_) ->
    next_endpoint_id(ESG,E) ;
    E = E0 ).

next_node_id(ESG,N) :-
  random_id(N0),
  ( esg_endpoint_node(ESG,_,N0) ->
    next_node_id(ESG,N) ;
    N = N0 ).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% ESG

% the set of nodes in this ESG
esg_nodes(ESG,Node_set) :-
  findall(N, esg_endpoint_node(ESG,_,N), Nodes),
  list_to_set(Nodes,Node_set).

% the direct successor of a node
esg_next_node(ESG,N0,N1) :- esg_edge(ESG,_,_,N0,N1).
% the direct predecessor of a node
esg_prev_node(ESG,N0,N1) :- esg_edge(ESG,_,_,N1,N0).

% unifies all nodes earlier then given node
esg_leq_node(ESG,N,Leq) :-
  ( Leq=N ; esg_leq_node_(ESG,N,Leq) ).
esg_leq_node_(ESG,N,Leq) :-
  esg_prev_node(ESG,N,Prev),
  ( Leq=Prev ; esg_leq_node_(ESG,Prev,Leq) ).

% find a path between nodes
esg_path(_ESG,N,N,[N]) :- !.
esg_path(ESG,N0,NX,[N0|Rest]) :-
  esg_next_node(ESG,N0,N1),
  esg_path(ESG,N1,NX,Rest).

initial_node(ESG,InitialNode) :-
  esg_nodes(ESG,Nodes),
  member(InitialNode,Nodes),
  \+ esg_edge(ESG,_,_,_,InitialNode).

% strip endpoint term
endpoint_type(-(Iri),Iri) :- !.
endpoint_type(+(Iri),Iri) :- !.
endpoint_type(Iri,Iri) :- atom(Iri), !.

esg_endpoint(ESG,Endpoint,Term,Node) :-
  esg_endpoint(ESG,Endpoint,Term),
  esg_endpoint_node(ESG,Endpoint,Node).

esg_edge(ESG,E0,E1,N0,N1) :-
  ( ground(N0) -> esg_endpoint_node(ESG,E0,N0) ; true ),
  ( ground(N1) -> esg_endpoint_node(ESG,E1,N1) ; true ),
  esg_edge(ESG,E0,E1),
  ( var(N0) -> esg_endpoint_node(ESG,E0,N0) ; true ),
  ( var(N1) -> esg_endpoint_node(ESG,E1,N1) ; true ).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% ESG manipulation

% establish `N0 = N1`
merge_nodes(ESG,N0,N1) :-
  findall(E, esg_endpoint_node(ESG,E,N1), Endpoints1),
  retractall(esg_endpoint_node(_,_,N1)),
  forall(member(E,Endpoints1),
         assertz(esg_endpoint_node(ESG,E,N0))).

% push `E0 < E1`
push_edge(ESG,Term0,Term1) :-
  % do not add if there is already a path from E0 to E1
  esg_endpoint(ESG,E0,Term0,N0),
  esg_endpoint(ESG,E1,Term1,N1),
  ( esg_edge(ESG,E0,E1) ; 
    esg_path(ESG,N0,N1,_) ), !.
push_edge(ESG,Term0,Term1) :-
  % contradiction in axiomatization
  esg_endpoint(ESG,_,Term0,N0),
  esg_endpoint(ESG,_,Term1,N1),
  esg_path(ESG,N1,N0,_),
  throw_axiom_contradiction(<(Term0,Term1)), !.
push_edge(ESG,Term0,Term1) :-
  esg_endpoint(ESG,E0,Term0,A),
  esg_endpoint(ESG,E1,Term1,B),
  forall((
    esg_leq_node(ESG,A,D),            % <*(D,A)
    esg_edge(ESG,E_D,E_C,D,C),        % <(D,C)
    esg_path(ESG,B,C,_)),             % <*(B,C)
    retractall(esg_edge(ESG,E_D,E_C)) % - (D,C)
  ),
  assertz(esg_edge(ESG,E0,E1)), !.    % + (A,B)
push_edge(ESG,Term0,Term1) :-
  ( \+ esg_endpoint(ESG,_,Term0) ->
    throw_unknown_endpoint(Term0) ; true ),
  ( \+ esg_endpoint(ESG,_,Term1) ->
    throw_unknown_endpoint(Term1) ; true ),
  fail.

% push an endpoint with a random
% id on the ESG
push_endpoint(ESG,Term,E) :-
  next_endpoint_id(ESG,E),
  next_node_id(ESG,N),!,
  assertz(esg_endpoint(ESG,E,Term)),
  assertz(esg_endpoint_node(ESG,E,N)).

% assert -(EvtType) and +(EvtType) endpoints
push_event_endpoints(ESG,EvtType) :-
  push_endpoint(ESG,-(EvtType),E0),
  push_endpoint(ESG,+(EvtType),E1),
  assertz(esg_edge(ESG,E0,E1)).      % -(EvtType) < +(EvtType)

% push another constraint on the ESG.
% this is either that two endpoints are equal,
% or that one is smaller then the other.
push_constraint(ESG,=(Term0,Term1)) :-
  % do not add if both are already in the same node
  esg_endpoint(ESG,_,Term0,N),
  esg_endpoint(ESG,_,Term1,N),!.
push_constraint(ESG,=(Term0,Term1)) :-
  once(esg_endpoint(ESG,E0,Term0,N0) ; throw_unknown_endpoint(Term0)),
  once(esg_endpoint(ESG,E1,Term1,N1) ; throw_unknown_endpoint(Term1)),
  % remove edges if there is an edge from some node N to one of the merged
  % nodes and if the other merged node is reachable from N.
  % then remove the edge from N to one of the merged nodes.
  forall((
    (( M=N0, K=N1 ) ; ( M=N1, K=N0 )),
    esg_edge(ESG,E_N,E_M,N,M),
    esg_path(ESG,N,K,_)),
    retractall(esg_edge(ESG,E_N,E_M))),
  % remove duplicate edges
  forall((
    esg_edge(ESG,E0,_,_,M),
    esg_edge(ESG,E1,EX1,_,M)),
    retractall(esg_edge(ESG,E1,EX1))),
  forall((
    esg_edge(ESG,_,E0,M,_),
    esg_edge(ESG,EX1,E1,M,_)),
    retractall(esg_edge(ESG,EX1,E1))),
  % push `E0 = E1`
  merge_nodes(ESG,N0,N1).
push_constraint(ESG,<(Term0,Term1)) :-
  push_edge(ESG,Term0,Term1).

% =(A,B) -> =(A-,B-), =(A+,B+)
push_allen_constraint(ESG,=(A,B)) :-
  push_constraint(ESG,=(-(A),-(B))),
  push_constraint(ESG,=(+(A),+(B))), !.

% <(A,B) -> <(A+,B-)
push_allen_constraint(ESG,<(A,B)) :-
  push_constraint(ESG,<(+(A),-(B))), !.
push_allen_constraint(ESG,>(A,B)) :-
  push_allen_constraint(ESG,<(B,A)), !.

% m(A,B) -> =(A+,B-)
push_allen_constraint(ESG,m(A,B)) :-
  push_constraint(ESG,=(+(A),-(B))), !.
push_allen_constraint(ESG,mi(A,B)) :-
  push_allen_constraint(ESG,m(B,A)), !.

% o(A,B) -> <(A-,B-), <(B-,A+), <(A+,B+)
push_allen_constraint(ESG,o(A,B)) :-
  push_constraint(ESG,<(-(A),-(B))),
  push_constraint(ESG,<(-(B),+(A))),
  push_constraint(ESG,<(+(A),+(B))), !.
push_allen_constraint(ESG,oi(A,B)) :-
  push_allen_constraint(ESG,o(B,A)), !.

% s(A,B) -> =(A-,B-), <(A+,B+)
push_allen_constraint(ESG,s(A,B)) :-
  push_constraint(ESG,=(-(A),-(B))),
  push_constraint(ESG,<(+(A),+(B))), !.
push_allen_constraint(ESG,si(A,B)) :-
  push_allen_constraint(ESG,s(B,A)),!.

% f(A,B) -> <(B-,A-), =(A+,B+)
push_allen_constraint(ESG,f(A,B)) :-
  push_constraint(ESG,=(+(A),+(B))),
  push_constraint(ESG,<(-(B),-(A))), !.
push_allen_constraint(ESG,fi(A,B)) :-
  push_allen_constraint(ESG,f(B,A)),!.

% NOTE: Allen defined during as:
%    B- < A-  , B+ <= A+  OR
%    B- <= A- , B+ < A 
% Which includes 'finishes' (f) and 'starts' (s).
% Here we use: `B- < A-  , B+ < A+`
push_allen_constraint(ESG,d(A,B)) :-
  push_constraint(ESG,<(-(B),-(A))),
  push_constraint(ESG,<(+(A),+(B))).
push_allen_constraint(ESG,di(A,B)) :-
  push_allen_constraint(ESG,d(B,A)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%

delete_node(ESG,N) :-
  forall(esg_endpoint_node(ESG,E,N), (
    retractall(esg_edge(ESG,E,_)),
    retractall(esg_edge(ESG,_,E)),
    retractall(esg_endpoint(ESG,E,_))
  )),
  retractall(esg_endpoint_node(ESG,_,N)).

delete_endpoint(ESG,Endpoint) :-
  retractall(esg_edge(ESG,_,Endpoint)),
  retractall(esg_edge(ESG,Endpoint,_)),
  retractall(esg_endpoint(ESG,Endpoint,_)),
  retractall(esg_endpoint_node(ESG,Endpoint,_)).

delete_nodes_before(ESG,N0) :-
  ( esg_prev_node(ESG,N0,N1) *->
    delete_nodes_before_(ESG,N1) ; true ).
delete_nodes_before_(ESG,N0) :-
  ( esg_prev_node(ESG,N0,N1) *->
    delete_nodes_before_(ESG,N1) ; true ),
  delete_node(ESG,N0).

delete_nodes_after(ESG,N0) :-
  ( esg_next_node(ESG,N0,N1) *->
    delete_nodes_after_(ESG,N1) ; true ).
delete_nodes_after_(ESG,N0) :-
  ( esg_next_node(ESG,N0,N1) *->
    delete_nodes_after_(ESG,N1) ; true ),
  delete_node(ESG,N0).

pull_out(ESG,N,E0,N0) :-
  retract(esg_endpoint_node(ESG,E0,N)),
  next_node_id(ESG,N0),
  assertz(esg_endpoint_node(ESG,E0,N0)).

pull_out_before(ESG,N,E0) :-
  esg_endpoint(ESG,E0,Term0,N),
  (( esg_endpoint_node(ESG,E1,N), E0 \= E1 ) -> (
     esg_endpoint(ESG,E1,Term1),
     forall(esg_edge(ESG,E0,X), (
            assertz(esg_edge(ESG,E1,X)),
            retractall(esg_edge(ESG,E0,X)))),
     forall((esg_endpoint_node(ESG,E2,N),
             esg_edge(ESG,X,E2)),
             delete_endpoint(ESG,X)),
     pull_out(ESG,N,E0,_N0),
     push_edge(ESG,Term0,Term1)) ;
     true ).

pull_out_after(ESG,N,E0) :-
  esg_endpoint(ESG,E0,Term0,N),
  (( esg_endpoint_node(ESG,E1,N), E0 \= E1 ) -> (
     esg_endpoint(ESG,E1,Term1),
     forall(esg_edge(ESG,X,E0), (
            assertz(esg_edge(ESG,X,E1)),
            retractall(esg_edge(ESG,X,E0)))),
     forall((esg_endpoint_node(ESG,E2,N),
             esg_edge(ESG,E2,X)),
             delete_endpoint(ESG,X)),
     pull_out(ESG,N,E0,_N0),
     push_edge(ESG,Term1,Term0)) ;
     true ).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ESG list operations

% pop out next endoint of the ESG.
% may yield multiple possible next endpoints
esg_pop([Node|Xs],Endpoint,RestESG) :-
  ( ground(Endpoint) -> Limit=true ; Limit=false ),
  % get random element of Node
  select(X,Node,NodeRest),
  % X is either a nested ESG (is_list) or an endpoint
  ( is_list(X) ->
    esg_pop(X,Endpoint,ESG_X) ;
    ( Endpoint = X, ESG_X = [] )
  ),
  % the remaining ESG
  ( ESG_X = [] ->
    SeqRest = NodeRest ;
    SeqRest = [ESG_X|NodeRest] ),
  ( SeqRest=[] ->
    RestESG=Xs ;
    RestESG=[SeqRest|Xs] ),
  % eliminate choicepoints of `select` in case a specific
  % next Endpoint was requested
  ( Limit=true -> ! ; true ).
% peak next endoint of the ESG.
% may yield multiple possible next endpoints
esg_peak([Node|_],Endpoint) :-
  member(X,Node),
  ( is_list(X) ->
    esg_peak(X,Endpoint) ;
    Endpoint=X ).

% Find path to the next Endpoint. This fails if there is none.
path_to_endpoint_dl(ESG,Endpoint,Zs-Zs,ESG) :-
  esg_peak(ESG,Endpoint),!.
path_to_endpoint_dl([X|Xs],Endpoint,[X|Ys]-Zs,RestESG) :-
  path_to_endpoint_dl(Xs,Endpoint,Ys-Zs,RestESG).

path_to_endpoint(ESG,Endpoint,Path,RestESG) :-
  path_to_endpoint_dl(ESG,Endpoint,Path_dl-Z,RestESG),
  append_dl(Path_dl-Z, []-[], Path-[]).

% ESG starts with -(Act), ActESG is a ESG
% from -(Act) to +(Act). 'join' both ESGs.
esg_join(ESG,[Tsk0,TskESG0],Joined) :-
  % pop out the -(Tsk) node, avoiding complications if -(Act)
  % is in a parallel node in one of the ESGs.
  esg_pop(TskESG0,-(Tsk0),TskESG1),
  esg_pop(ESG,-(Tsk1),ESG1),
  ( ( Tsk0 = Tsk1 );
    ( has_type(Tsk0,TskType), has_type(Tsk1,TskType) )
  ), !,
  % find a path to +(Act) in both ESGs
  % NOTE: the paths are represented as difference list to allow
  %       for performing `append` in constant time.
  path_to_endpoint_dl(TskESG1,+(Tsk0),Path0_dl-Zs0,[TskNode1|_]),
  path_to_endpoint_dl(ESG1,   +(Tsk1),Path1_dl-Zs1,[TskNode2|Rest]),
  % Merge nodes of both graphs that contain +(Tsk).
  % This is needed in case +(Tsk) is in a "parallel node" in at least
  % one of the graphs.
  append(TskNode1,TskNode2,TskNode3),
  once(select(+(Tsk1),TskNode3,TskNode4)),
  RestESG=[TskNode4|Rest],
  % prefix ESG is -(Act) followed by a path to +(Act),
  % here represented as difference list
  ( Path0_dl=[Zs0] -> PrefixESG=[[-(Tsk0)]|Path1_dl]-Zs1 ;
    Path1_dl=[Zs1] -> PrefixESG=[[-(Tsk0)]|Path0_dl]-Zs0 ; (
    % eliminate variables of difference lists
    Zs0=[],Zs1=[],
    % parallelize the paths
    % NOTE: we assume here that there are no shared endpoints in both paths.
    %       This requires that:
    %       - actions must not be axiomatized by what action follows or preceeds.
    %       - complex actions only have sub-actions, simple actions only have phases.
    %         i.e., NO has-phase axioms for complex-actions, and also
    %         NO allen constraints to processes, only to sub-actions.
    Path=[Path0_dl,Path1_dl],
    PrefixESG=[[-(Tsk0)],Path|Zs2]-Zs2 )),
  % constant time append
  append_dl(PrefixESG, RestESG-[], Joined-[]).


esg_endpoints(ESG,Endpoints) :-
  esg_endpoints_(ESG->[],[]->Endpoints).
esg_endpoints_([]->[],X->X) :- !.
esg_endpoints_(ESG_0->ESG_n,Prev->Ys) :-
  esg_pop(ESG_0,E_0,ESG_1),
  endpoint_type(E_0,Type),
  \+ contains_endpoint_type(Prev,Type), !,
  esg_endpoints_(ESG_1->ESG_n,[E_0|Prev]->Ys).
esg_endpoints_(ESG_0->ESG_n,Prev->Ys) :-
  esg_pop(ESG_0,_,ESG_1),
  esg_endpoints_(ESG_1->ESG_n,Prev->Ys).

contains_endpoint_type([E|_],Type) :-
  endpoint_type(E,Type),!.
contains_endpoint_type([_|Es],Type) :-
  contains_endpoint_type(Es,Type).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% List-based representation

% split one sequence at a needle node (X)
split_at([X|Xs],X,[],[X|Xs]) :- !.
split_at([Y|Ys],X,[Y|Left],Right) :-
  split_at(Ys,X,Left,Right).

% split two sequences at the position where they share first node
split_at_equal([A|As],Bs,[],Left_B,[A|As],Right_B) :-
  split_at(Bs,A,Left_B,Right_B), !.
split_at_equal([A|As],Bs,[A|A_Left],Left_B,Right_A,Right_B) :-
  split_at_equal(As,Bs,A_Left,Left_B,Right_A,Right_B).

% merge sequences up to where they are equal
merge_equal([],Bs,[],[],Bs) :- !.
merge_equal(As,[],[],As,[]) :- !.
merge_equal([],[],[],[],[]) :- !.
merge_equal([A|As],[A|Bs],[A|Equal],Right_A,Right_B) :-
  merge_equal(As,Bs,Equal,Right_A,Right_B), !.
merge_equal([A|As],[B|Bs],[],[A|As],[B|Bs]) :- A \= B, !.

% split two sequences into sub-sequences where they are
% equal and where they are not equal
merge_sequences([],[],[]) :- !.
merge_sequences([],Bs,Bs) :- !.
merge_sequences(As,[],As) :- !.
merge_sequences([A0|As],[A0|Bs],Merged) :-
  merge_equal([A0|As],[A0|Bs],EqualNodes,Right_A,Right_B),
  merge_sequences(Right_A,Right_B,Rest),
  (( Rest=[] ; Rest=[[]] ) ->
    Merged=EqualNodes ;
    append(EqualNodes,Rest,Merged) ), !.
merge_sequences([A0|As],[B0|Bs],Rest_X) :-
  A0 \= B0,
  ( split_at_equal([A0|As],[B0|Bs],Left_A,Left_B,Right_A,Right_B) ->
    merge_sequences(Right_A,Right_B,Rest) ; (
    % none endpoints are shared in the sequences
    Left_A=[A0|As],
    Left_B=[B0|Bs],
    Rest=[] )),
  ( Left_B=[] ->
    Left=Left_A ;
    Left=[[Left_A,Left_B]] ),
  append(Left,Rest,Rest_X).

merge_sequences2([],[]) :- !.
merge_sequences2([X],X) :- !.
merge_sequences2([X|Xs],Blend) :-
  merge_sequences2(Xs,Xs_blend),
  merge_sequences(X,Xs_blend,Blend).

extract_node_sequence_reverse(ESG,N0,Sequence) :-
  % move to previous node(s), find sequences starting from each next node
  findall([N0|N1_sequence],(
    esg_prev_node(ESG,N0,N1) *->
    extract_node_sequence_reverse(ESG,N1,N1_sequence) ;
    N1_sequence=[]
    ),
    ParallelSequences),
  merge_sequences2(ParallelSequences,Sequence).

extract_node_sequence(ESG,N0,Sequence) :-
  % move to next node(s), find sequences starting from each next node
  findall([N0|N1_sequence],(
    esg_next_node(ESG,N0,N1) *->
    extract_node_sequence(ESG,N1,N1_sequence) ;
    N1_sequence=[]
    ),
    ParallelSequences),
  merge_sequences2(ParallelSequences,Sequence).

extract_node_sequence(ESG,Sequence) :-
  % find a node sequence starting from each initial node
  findall(S, (
    initial_node(ESG,N),
    extract_node_sequence(ESG,N,S)),
    Sequences),
  merge_sequences2(Sequences,Sequence).

%% esg_to_list(+ESG,?Sequence).
%
esg_to_list(ESG,Sequence) :-
  % find a possible sequence covering all nodes of the ESG
  extract_node_sequence(ESG,NodeSequence),
  % expand nodes to lists of endpoint terms associated to the nodes
  esg_to_list_(ESG,NodeSequence,Sequence).

esg_to_list_(_ESG,[],[]) :- !.
esg_to_list_(ESG,[[N]|Ns],[NEs|Es]) :-
  esg_to_list_(ESG,[N|Ns],[NEs|Es]), !.
esg_to_list_(ESG,[N|Ns],[NEs|Es]) :-
  is_list(N), !,
  findall(Seq, (
    member(N0,N),
    esg_to_list_(ESG,N0,Seq)), NEs),
  esg_to_list_(ESG,Ns,Es).
esg_to_list_(ESG,[N|Ns],[NEs|Es]) :-
  findall(E_term, (
    esg_endpoint_node(ESG,E,N),
    esg_endpoint(ESG,E,E_term)), NEs),
  esg_to_list_(ESG,Ns,Es).


pre_event_graph(ESG,N,PreSequence) :-
  findall(N1_sequence,(
    esg_prev_node(ESG,N,N1),
    extract_node_sequence_reverse(ESG,N1,N1_sequence)),
    ParallelSequences),
  merge_sequences2(ParallelSequences,NodeSequence),
  esg_to_list_(ESG,NodeSequence,PreSequence).

post_event_graph(ESG,N,PostSequence) :-
  findall(N1_sequence,(
    esg_next_node(ESG,N,N1),
    extract_node_sequence(ESG,N1,N1_sequence)),
    ParallelSequences),
  merge_sequences2(ParallelSequences,NodeSequence),
  esg_to_list_(ESG,NodeSequence,PostSequence).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Asserting / Retracting ESGs

%% esg(+Events,+Constraints,-ESG).
%
esg_assert(Events,Constraints,ESG) :-
  esg_unique_id(ESG),
  % assert two endpoints for each event,
  % and an edge from -(Evt) to +(Evt)
  forall(member(Evt,Events),
         push_event_endpoints(ESG,Evt)),
  % push temporal constraints on the ESG causing
  % new edges to be created and also some edges to be removed
  forall(member(C,Constraints),
         push_allen_constraint(ESG,C)), !.

%% esg_retract(+ESG).
%
esg_retract(ESG) :-
  retractall(esg_edge(ESG,_,_)),
  retractall(esg_endpoint(ESG,_,_)),
  retractall(esg_endpoint_node(ESG,_,_)).

%% esg(+Act,+Events,+Constraints,?Sequence).
%
esg(Act,Events,Constraints,Sequence) :-
  list_to_set([Act|Events],EvSet),
  esg_assert(EvSet,Constraints,ESG),
  %esg_write_info(ESG),
  esg_to_list(ESG,Sequence),
  esg_retract(ESG).

esg_truncated(Act,Events,Constraints,[Sequence,PreESG,PostESG]) :-
  list_to_set([Act|Events],EvSet),
  esg_assert(EvSet,Constraints,ESG),
  esg_truncate(ESG,Act,PreESG,PostESG),
  %esg_write_info(ESG),
  esg_to_list(ESG,Sequence),
  esg_retract(ESG).

%% esg_event_sequence(+ESG,?Sequence) is semidet.
%
%
esg_event_sequence(ESG0,[Evt|Xs]) :-
  esg_pop(ESG0,-(Evt),ESG1),!,
  esg_event_sequence(ESG1,Xs).

esg_event_sequence(ESG0,Xs) :-
  esg_pop(ESG0,+(_),ESG1),!,
  esg_event_sequence(ESG1,Xs).

esg_event_sequence([],[]).

% NOTE: Actions MUST contain their sub-actions (i.e., no overlapping etc.).
%       Else we may remove action endpoints here needed for the action parser.
esg_truncate(ESG,Evt,PreESG,PostESG) :-
  esg_endpoint(ESG,E0,-(Evt)),
  esg_endpoint_node(ESG,E0,N0),
  esg_endpoint(ESG,E1,+(Evt)),
  esg_endpoint_node(ESG,E1,N1),
  % store pre- and post-condition ESGs
  pre_event_graph(ESG,N0,PreESG),
  post_event_graph(ESG,N1,PostESG),
  % delete nodes before -Evt, and move -Evt into its own node.
  delete_nodes_before(ESG,N0),
  pull_out_before(ESG,N0,E0),
  % delete nodes after +Evt, and move +Evt into its own node.
  delete_nodes_after(ESG,N1),
  pull_out_after(ESG,N1,E1), !.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Debugging

write_concept(Concept) :- write(Concept).
  
endpoint_write(-(Iri)) :-
  write('-'), write_concept(Iri).
endpoint_write(+(Iri)) :-
  write('+'), write_concept(Iri).

esg_write(ESG) :-
  write('['), esg_write_(ESG), write(']').
esg_write_([]) :- !.
esg_write_([X|Xs]) :-
  endpoint_write(X),
  ( Xs=[] -> true ; write(' ') ),
  esg_write_(Xs).
esg_write_([X|Xs]) :-
  is_list(X),
  esg_write_(X),
  ( Xs=[] -> true ; write(' ') ),
  esg_write_(Xs).

esg_write_edges(ESG) :-
  forall((
      esg_edge(ESG,E0,E1),
      esg_endpoint(ESG,E0,Term0),
      esg_endpoint(ESG,E1,Term1)),(
      endpoint_write(Term0),
      write(' < '),
      endpoint_write(Term1),
      nl )).

esg_write_count(ESG) :-
  findall((X,Y), esg_edge(ESG,X,Y), Edges),
  findall(X, esg_endpoint(ESG,X,_), Endpoints),
  esg_nodes(ESG,Nodes),
  length(Edges,NumEdges),
  length(Endpoints,NumEndpoints),
  length(Nodes,NumNodes),
  atomic_list_concat(['      [esg.pl]',
      ' num endpoints: ', NumEndpoints,
      ' num nodes: ', NumNodes,
      ' num edges: ', NumEdges
  ], Msg2),
  writeln(Msg2).

esg_write_info(ESG) :-
  esg_write_count(ESG)
  %, esg_write_edges(ESG)
  .
