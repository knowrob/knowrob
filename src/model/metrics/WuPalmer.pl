:- module(metrics_WuPalmer,
    [ rdf_wup_similarity/3,
      rdf_wup_similarity_given_LCS/4,
      rdf_path_distance/3,
      rdf_shortest_path/3,
      rdf_paths/3,
      rdf_most_similar/4,
      rdf_all_similar/3,
      rdf_common_ancestor/2
    ]).
/** <module> Predicates that calculate semantic similarities between objects.

@author Moritz Tenorth
@author Lars Kunze
@author Martin Schuster
@license BSD
*/

:- use_module(library('semweb/rdf_db'), [rdf_meta/1]).

:- rdf_meta(rdf_wup_similarity(r,r,-)).
:- rdf_meta(rdf_wup_similarity_given_LCS(r,r,r,-)).
:- rdf_meta(rdf_path_distance(r,r,-)).
:- rdf_meta(rdf_shortest_path(r,r,-)).
:- rdf_meta(rdf_paths(r,r,t)).
:- rdf_meta(rdf_most_similar(r,r,+,-)).
:- rdf_meta(rdf_all_similar(r,r,-)).
:- rdf_meta(rdfs_common_ancestor(t,r)).
:- rdf_meta(rdf_shortest_dist_up(r,r,?)).
:- rdf_meta(rdf_paths_up(r,r,t)).

%% rdf_wup_similarity(+A:rdf_class, +B:rdf_class, -Sim:float).
%
% Calculates WUP similarty between two classes
% 
% similarity = depth(LeastCommonSuperclass) / (1/2 * (depth(A) + depth(B)))
% 
% similarity of A and A is 1
%
% @param A one of the two classes
% @param B second of the two classes
% @param Sim similarty measure between 0 (not similar) and 1 (most similar)
% 
rdf_wup_similarity(A, A, 1) :- !.
rdf_wup_similarity(A, B, Sim) :-
    rdf_common_ancestor([A, B], LCS),!,
    rdf_wup_similarity_given_LCS(A, B, LCS, Sim).


%% rdf_wup_similarity_given_LCS(+A:rdf_class, +B:rdf_class, +LCS:rdf_class, -Sim:float).
%
% Calculates WUP similarty between two classes using a given least common ancestor
% 
% similarity = depth(LCS) / (1/2 * (depth(A) + depth(B)))
% 
% similarity of A and A is 1
%
% @param A one of the two classes
% @param B second of the two classes
% @param LCS least common ancestor
% @param Sim similarty measure between 0 (not similar) and 1 (most similar)
% 
rdf_wup_similarity_given_LCS(A, B, LCS, Sim) :-
    rdf_shortest_dist_up(LCS, dul:'Entity', DepthLCS),
    rdf_shortest_dist_up(A,   LCS, DepthA),
    rdf_shortest_dist_up(B,   LCS, DepthB),
    Sim is 2*DepthLCS / (DepthA + DepthB + 2*DepthLCS).

 
%% rdf_shortest_dist_up(+A, +B, -Dist).
%
% Searches the shortest distance of a directed path from A to B
%
% @param A    OWL class
% @param B    OWL class
% @param Dist Shortest distance from A to B, will be false if no such path exists
% 
rdf_shortest_dist_up(A, B, Dist) :-
    rdf_shortest_dist_up_superclasses([A], B, [], Dist).

%% rdf_shortest_dist_up_superclasses(+SearchList, +Goal, +DoneList, -Dist).
%
% Searches the shortest distance of a directed path from any class in SearchList to the class Goal.
%
% @param SearchList List of OWL classes
% @param Goal       OWL class
% @param DoneList   Contains all classes searched so far, they will be excluded from the search
% @param Dist       Shortest distance from any class in SearchList to Goal, will be false if no such path exists
% 
rdf_shortest_dist_up_superclasses(SearchList, Goal, _, 0) :-
    member(Goal, SearchList), !.
rdf_shortest_dist_up_superclasses(SearchList, Goal, DoneList, Dist) :-
    findall(NewSuperClasses, (
        member(Class, SearchList),
        findall(SuperClass, (
            (SuperClass=Class; subclass_of(Class,SuperClass)),
            \+ is_restriction(SuperClass),
            \+ member(SuperClass, DoneList)
        ), NewSuperClasses)
    ), NewSuperClassesLists),
    flatten(NewSuperClassesLists, NewSearchList),
    (  NewSearchList = [] %if no new classes in searchlist, then Goal is not reachable -> return false
    -> false  %done
    ;  append(DoneList, SearchList, NewDoneList),
       rdf_shortest_dist_up_superclasses(NewSearchList, Goal, NewDoneList, NewDist),
       Dist is NewDist + 1
    ).

%% rdf_path_distance(+A, +B, -Dist).
%
% Compute the path distance between A and B, i.e. the inverse of the length of the shortest path
%
% @param A      OWL class
% @param B      OWL class
% @param Paths  List of elements along the path between A and B
% 
rdf_path_distance(A, B, Dist) :-
  rdf_shortest_path(A, B, Shortest),
  Dist is 1 / Shortest.

%% rdf_shortest_path(+A, +B, -Dist).
%
% Compute the length of the shortest path between A and B
%
% @param A      OWL class
% @param B      OWL class
% @param Paths  Lenght of the shortest path between A and B
% 
rdf_shortest_path(A, B, Dist) :-
  findall(Path, rdf_paths(A, B, Path), Paths),
  maplist(length, Paths, [L|Ls]),
  foldl(num_num_min, Ls, L, Dist).

num_num_min(X,Y,Min) :- Min is min(X,Y).

%% rdf_paths(+A, +B, -Paths).
%
% Compute path between classes A and B
%
% @param A      OWL class
% @param B      OWL class
% @param Paths  List of elements along the path between A and B
% 
rdf_paths(A, B, [A|Path]) :-

  % go upwards, and for all paths check if you have a direct way down to the goal
  rdf_paths_up(A, dul:'Entity', U),
  rdf_find_path_down(U, B, Path).

rdf_find_path_down([UPa|_], B, [UPa|D]) :-
  rdf_paths_down(UPa, B, D).

rdf_find_path_down([UPa|Up], B, [UPa|D]) :-
  \+ rdf_paths_down(UPa, B, D),
  rdf_find_path_down(Up, B, D).

rdf_find_path_down([], _, []).

rdf_paths_up(A, A, []).
rdf_paths_up(A, B, [Super|Path]) :-
  findall(C, subclass_of(A,C), Cs), member(Super, Cs),
  rdf_paths_up(Super, B, Path).

rdf_paths_down(A, A, []).
rdf_paths_down(A, B, [Sub|Path]) :-
  findall(C, subclass_of(C,A), Cs), member(Sub, Cs),
  rdf_paths_down(Sub, B, Path).

%% rdf_most_similar(Class, Super, N, NMostSim).
%
% Find the N sub-classes of Super that are most similar to Class
%
% @param Class     Class to be considered
% @param Super     Common super-class
% @param N         Number of similar classes to be computed
% @param NMostSim  List of the N most similar classes
%
rdf_most_similar(Class, Super, N, NMostSim) :-
  findall([A, D], (subclass_of(A, Super),
                   rdf_wup_similarity(A, Class, D)), Dists),
  predsort(compare_inference_probs, Dists, MostSim),
  first_n_elem(MostSim, N, NMostSim).



%% rdf_common_ancestor(+Classes, ?Ancestor).
%
% Get one of the most specific common ancestors of rdf_classes.
%
rdf_common_ancestor([C], C).
rdf_common_ancestor([C1, C2| Cs], C) :-
  rdf_superclass_list([C1, C2| Cs], SCs),
  intersection_of_sets(SCs, CSCs),
  most_specific_class(CSCs, C0),
  C = C0.

		 /*******************************
		 *	  Utility predicates		*
		 *******************************/

rdf_superclass_list([], []).
rdf_superclass_list([C|CRest], [SCs| SCRest]) :-
  findall(SC, (SC=C ; subclass_of(C,SC)), SCs),
  rdf_superclass_list(CRest, SCRest).

intersection_of_sets([], []).
intersection_of_sets([L], L).
intersection_of_sets([L0, L1|LRest], Intersection) :-
  intersection(L0, L1, L),
  intersection_of_sets([L|LRest], Intersection).

most_specific_class([C], C).
most_specific_class([C1,C2|Cs], C) :-
  subclass_of(C2, C1)
  -> most_specific_class([C2|Cs], C)
  ; most_specific_class([C1|Cs], C). % Either not comparable or C2 is superclass of C1


rdf_all_similar(Class, Super, MostSim) :-
  findall([A, D], (subclass_of(A, Super),
                   rdf_wup_similarity(A, Class, D)), Dists),
  predsort(compare_inference_probs, Dists, MostSim).

first_n_elem([F|In], N, [F|Out]) :-
    N>0,
    N1 is N-1,
    first_n_elem(In, N1, Out),!.

first_n_elem(_, N, []) :-
    N=<0,!.

compare_inference_probs('>', [_, P1], [_, P2]) :-
    term_to_atom(N1, P1),
    term_to_atom(N2, P2),
    N1 < N2.

compare_inference_probs('<', [_, P1], [_, P2]) :-
    term_to_atom(N1, P1),
    term_to_atom(N2, P2),
    N1>=N2.
