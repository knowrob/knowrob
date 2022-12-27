:- module(mongolog_watch,
    [ watch(t,t,-),
      watch_event(+,+),
      unwatch(+)
    ]).

:- use_module(library('mongodb/client')).

%% watcher(+WatcherID, +Goal)
%
% Stores information about active watch processes.
%
:- dynamic watcher/2.

%% watch(+Goal, +Callback, -WatcherID) is semidet.
%
% Start watching possible instantiations of variables in Goal.
% Goal must be a KnowRob language term.
% Callback is called whenever the set of possible instantiations changes,
% and it is provided with information about the change.
% The change information is encoded in a term that is appended to
% already existing arguments of Callback, if any.
%
% Note that callback is currently not called when documents are removed!
% This might change in the future.
%
% Also note that currently Goal must be a term triple/3.
% The scope of triples is ignored.
% No other predicates are supported yet.
% This might also change in the future.
%
% @param Goal a KnowRob language term with free variables.
% @param Callback a predicate called for each change event.
% @param WatcherID a unique identifier of the watching operation.
%
watch(Goal, Callback, WatcherID) :-
	% create match filter based on Goal
	% NOTE: it is not possible to $match remove events like this
	%       because the document values are _not_ included in the event :/
	%       this means additional bookkeeping is needed to be able to handle
	%       remove events ...
	watch_filter(Goal, DB, Coll, WatchFilter),
	% start receiving events about documents that match the filter
	mng_watch(DB, Coll,
		watch_event,
		[pipeline, array([
			['$match', WatchFilter]
		])],
		WatcherID),
	% FIXME: there is a risk watch_event is called
	%        before this fact is asserted
	assertz(watcher(WatcherID, Callback)).

%% unwatch(+WatcherID) is semidet.
%
% Stop a previously started watch operation.
% This will fail if WatcherID does not belong to
% a active watch operation.
%
% @param WatcherID a unique identifier of the watching operation.
%
unwatch(WatcherID) :-
	once(watcher(WatcherID, _)),
	mng_unwatch(WatcherID),
	retractall(watcher(WatcherID, _)).

%% watch_event(+WatcherID, +Event) is semidet.
%
% This predicate is called by the mongo client when
% changes are received in a watch operation.
% Event is a change stream response document.
%
% @param WatcherID a unique identifier of the watching operation.
% @param Event a change stream output document.
%
% @see https://docs.mongodb.com/manual/reference/change-events/#change-stream-output
%
watch_event(WatcherID, Event) :-
	once(watcher(WatcherID, CallbackGoal)),
	dict_pairs(Dict, _, Event),
	mng_get_dict(operationType, Dict, string(OpType)),
	memberchk(OpType, [insert, update]),
	% read the document from "fullDocument" field
	mng_get_dict(fullDocument,  Dict, FullDoc),
	mng_get_dict('_id', FullDoc, id(DocID)),
	% parse triple
	% TODO: support other terms here
	mng_get_dict(s, FullDoc, string(S)),
	mng_get_dict(p, FullDoc, string(P)),
	mng_get_dict(o, FullDoc, TypedValue),
	mng_strip_type(TypedValue, _, Value),
	% finally call the watcher
	UpdateTerm = event(OpType, [
		id(DocID),
		new_value(triple(S,P,Value))
	]),
	catch(
		call(CallbackGoal, UpdateTerm),
		Exc,
		log_error_and_fail(lang(Exc,
			watch_callback(CallbackGoal, UpdateTerm)))
	).

%%
watch_filter(triple(S,P,V), DB, Coll, WatchFilter) :-
	% TODO: howto handle scope of facts here?
	% TODO: support other terms here
	mng_get_db(DB, Coll, 'triples'),
	mongolog_triple:mng_triple_doc(triple(S,P,V), Doc, []),
	% the event holds the updated document in a field "fullDocument"
	% NOTE: not for remove events!
	maplist([[Key0,Val],[Key1,Val]]>>
		atom_concat('fullDocument.', Key0, Key1),
		Doc, WatchFilter).
