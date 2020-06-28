:- module(lang_is_at,
	[ is_at(r,t)  % ?Object, -PoseData
	]).
/** <module> The is_at predicate.

@author Daniel Beßler
@license BSD
*/

:- op(1000, xfx, user:is_at).

is_at(Object,PoseData) :-
	ask(is_at(Object,PoseData)).
