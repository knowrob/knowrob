:- module(tf_mongo,
	[ tf_mng_store/3,
	  tf_mng_lookup/6,
	  tf_mng_lookup_all/1,
	  tf_mng_lookup_all/2,
	  tf_mng_trajectory/4,
	  tf_mng_drop/0,
	  tf_mng_tree/2,
	  tf_mng_tree_lookup/3
	]).

:- use_foreign_library('libtf_knowrob.so').

:- use_module(library('semweb/rdf_db'),
	[ rdf_split_url/3 ]).
:- use_module(library('utility/algebra'),
	[ transform_between/3,
	  transform_multiply/3
	]).
:- use_module(library('db/mongo/client')).

% stores the last TF tree constructed from mongo
:- dynamic tree_cache/2.

% register tf_raw/9 as a fluent predicate
% FIXME: need to update collection name if collection prefix is changed
:- mng_get_db(_DB, CollectionName, 'tf'),
   mongolog_add_fluent(tf_raw,
	   	% predicate arguments
		[	+'child_frame_id'           % +ChildFrame
		,	-'header.frame_id'          % -ParentFrame
		,	-'transform.translation.x'  % -X
		,	-'transform.translation.y'  % -Y
		,	-'transform.translation.z'  % -Z
		,	-'transform.rotation.x'     % -QX
		,	-'transform.rotation.y'     % -QY
		,	-'transform.rotation.z'     % -QZ
		,	-'transform.rotation.w'     % -QW
		],
   		% time field
   		'header.stamp',
		% options
		[	collection(CollectionName)
		]).

% add *tf* as a mongolog command such that it can be
% used in mongolog queries.
:- mongolog:add_command(tf).

% tf/4
lang_query:step_expand(
		tf(ChildFrame, ParentFrame, [X,Y,Z], [QX,QY,QZ,QW]),
		tf_raw(ChildFrame, ParentFrame, X, Y, Z, QX, QY, QZ, QW)).

lang_query:step_expand(
		assert(tf(ChildFrame, ParentFrame, [X,Y,Z], [QX,QY,QZ,QW])),
		assert(tf_raw(ChildFrame, ParentFrame, X, Y, Z, QX, QY, QZ, QW))).

% tf/2
lang_query:step_expand(
		tf(ChildFrame, [ParentFrame, [X,Y,Z], [QX,QY,QZ,QW]]),
		tf_raw(ChildFrame, ParentFrame, X, Y, Z, QX, QY, QZ, QW)).

lang_query:step_expand(
		assert(tf(ChildFrame, [ParentFrame, [X,Y,Z], [QX,QY,QZ,QW]])),
		assert(tf_raw(ChildFrame, ParentFrame, X, Y, Z, QX, QY, QZ, QW))).

%%
tf_db(DB, Name) :-
	mng_get_db(DB, Name, 'tf').

%% tf_mng_drop is det.
%
% Drops all documents in the TF database collection.
%
tf_mng_drop :-
	tf_db(DB, Name),
	mng_drop(DB,Name).

%% tf_mng_lookup(+ObjFrame, +QuerySince, +QueryUntil, -PoseData, -FactSince, -FactUntil) is nondet.
%
% Retrieve all transforms of frame within some time interval.
%
tf_mng_lookup(
		ObjFrame, Query_Since, Query_Until,
		PoseData, Fact_Since, Fact_Until) :-
	% create a query scope
	time_scope(=<(Query_Since), >=(Query_Until), QueryScope),
	mongolog_call(
		tf(ObjFrame, PoseData),
		[ scope(QueryScope),
		  user_vars([['v_scope',FactScope]])
		]),
	% read values from FactScope
	time_scope(double(Fact_Since), double(Fact_Until), FactScope).

%% tf_mng_lookup(+ObjFrame, -PoseData) is nondet.
%
% Retrieve the latest transform of a frame.
%
tf_mng_lookup(ObjFrame, PoseData) :-
	get_time(Now),
	tf_mng_lookup(ObjFrame, Now, Now, PoseData, _, _).

%% tf_mng_store(+ObjFrame, +PoseData, +Stamp) is det.
%
% Store a transform in mongo DB.
%


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % trajectories

%% tf_mng_trajectory(+Obj, +Begin, +End, -Trajectory) is det.
%
% Read all transforms associated to a frame in given time interval.
%
tf_mng_trajectory(Obj,Stamp0,Stamp1,Trajectory) :-
	rdf_split_url(_,ObjFrame,Obj),
	findall(Stamp-Data,
		( tf_mng_lookup(ObjFrame,Stamp0,Stamp1,Data,Stamp,_),
		  Stamp >= Stamp0,
		  Stamp =< Stamp1
		),
		Trajectory0
	),
	reverse(Trajectory0,Trajectory).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % bulk lookup


%% tf_mng_lookup_all(-Transforms) is det.
%
% Retrieve latest transform of each frame.
%
tf_mng_lookup_all(Transforms) :-
	tf_mng_lookup_all(Transforms, _).


%% tf_mng_lookup_all(-Transforms, +Stamp) is det.
%
% Retrieve latest transform of each frame.
% Documents later than Stamp are ignored.
%
tf_mng_lookup_all(Transforms, Stamp) :-
	tf_db(DB,Coll),
	% first create lookup $match steps
	findall(MatchStep,
		(	MatchStep=['$eq',array([string('$child_frame_id'),string('$$frame')])]
		;	(	ground(Stamp),
				MatchStep=['$lt',array([string('$header.stamp'),time(Stamp)])]
			)
		),
		MatchSteps),
	% create aggregate pipeline
	% TODO: $facet may can be used to avoid $lookup here
	% TODO: integrate this with mongolog fluents?
	%       i.e. when key fields are queried, then $group them
	findall(Step,
		% get distinct frames
		(	Step=['$group',['_id',['child_frame_id',string('$child_frame_id')]]]
		;	Step=['$lookup',[
				['from',string(Coll)],
				['as',string('tf')],
				['let',['frame',string('$_id.child_frame_id')]],
				['pipeline',array([
					['$match',['$expr',['$and',array(MatchSteps)]]],
					['$sort',[
						['child_frame_id',int(1)],
						['header.stamp',int(-1)]
					]],
					['$limit',int(1)]
				])]
			]]
		;	Step=['$unwind',string('$tf')]
		;	Step=['$replaceRoot',['newRoot',string('$tf')]]
		),
		Pipeline
	),
	%%
	setup_call_cleanup(
		mng_cursor_create(DB,Coll,Cursor),
		(	mng_cursor_aggregate(Cursor,['pipeline',array(Pipeline)]),
			findall([Ref,Frame,Pos,Rot],
				(	mng_cursor_materialize(Cursor,Doc),
					tf_mng_doc_pose(Doc,Frame,_,[Ref,Pos,Rot])
				),
				Transforms
			)
		),
		mng_cursor_destroy(Cursor)
	).


%%
% Convert mongo document to pose term.
%
tf_mng_doc_pose(Doc,ObjFrame,Time,[ParentFrame,[TX,TY,TZ],[QX,QY,QZ,QW]]) :-
	get_dict(child_frame_id, Doc, string(ObjFrame)),
	get_dict(header,Doc,
		[ _, stamp-time(Time), frame_id-string(ParentFrame) ]
	),
	get_dict(transform,Doc,[
		translation-[ x-double(TX), y-double(TY), z-double(TZ) ],
		rotation-[ x-double(QX), y-double(QY), z-double(QZ), w-double(QW) ]
	]).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % TF tree


%% tf_mng_tree(+Stamp, -Tree) is semidet.
%
% Loads full TF tree for given timestamp.
% All poses in the tree are transformed into world coordinates.
%
tf_mng_tree(Stamp, Tree) :-
	tree_cache(Stamp, Tree),
	!.

tf_mng_tree(Stamp, Tree) :-
	% read all required transforms
	tf_mng_lookup_all(Transforms, Stamp),
	% create a tree structure for traversal operation
	tree_create(Transforms, Tree0),
	% compute world poses
	tree_traverse(Tree0, Tree1),
	% create dictionary for faster lookup
	tree_dict(Tree1, Tree),
	% cache tree
	retractall(tree_cache(_,_)),
	assertz(tree_cache(Stamp, Tree)).

%% tf_mng_tree_lookup(+Tree, +ChildFrame, -PoseData) is semidet.
%
% Retrieve pose of named frame from a previously created TF tree.
% PoseData is a list `[RefFrame,Position,Rotation]`.
% RefFrame maybe grounded in which case the requested relative
% transform is computed if needed and possible.
%
tf_mng_tree_lookup(Tree, ChildFrame, [RefFrame,Pos,Rot]) :-
	get_dict(ChildFrame, Tree, [RefFrame0,Pos0,Rot0]),
	% return identity of transform if possible
	(	RefFrame=RefFrame0 -> (	Pos=Pos0, Rot=Rot0 )
	% else compute transform wrt. requested reference frame
	;	(	get_dict(RefFrame, Tree, [RefFrame0,Pos1,Rot1]),
			transform_between(
				[RefFrame0,ChildFrame,Pos0,Rot0],
				[RefFrame0,RefFrame,Pos1,Rot1],
				[RefFrame,ChildFrame,Pos,Rot])
		)
	),
	!.

%%
tree_create(Transforms,Tree) :-
	% gather all childrens
	findall(Frame-[Parent,Pos,Rot,Children],
		(	member([Parent,Frame,Pos,Rot],Transforms),
			findall(X, member([Frame,X,_,_],Transforms), Children)
		),
		Frames
	),
	dict_pairs(Frames_dict,_,Frames),
	% create the tree
	findall(Branch,
		(	member([Root,Frame,_,_],Transforms),
		\+	get_dict(Root,Frames_dict,_),
			tree_create_branch(Frame,Frames_dict,Branch)
		),
		Tree
	).

%%
tree_create_branch(Frame,Dict,(Pose,Children)) :-
	get_dict(Frame,Dict,[Parent,Pos,Rot,Children0]),
	Pose=[Parent,Frame,Pos,Rot],
	findall(X,
		(	member(ChildFrame,Children0),
			tree_create_branch(ChildFrame,Dict,X)
		),
		Children).

%%
tree_traverse([], []) :- !.
tree_traverse([X|Xs], [Y|Ys]) :-
	tree_traverse_0(X,Y),
	tree_traverse(Xs,Ys).

%%
tree_traverse_0((Pose,[]), (Pose,[])):- !.
tree_traverse_0((Pose,[X|Xs]), (Pose,[Y|Ys])):-
	tree_traverse_1(Pose,X,Y),
	tree_traverse_0((Pose,Xs), (Pose,Ys)).

%%
tree_traverse_1(Pose_parent, (Pose_child,Xs), (Pose,Ys)) :-
	% get child pose in world frame
	transform_multiply(Pose_parent,Pose_child,Pose),
	% continue for children
	tree_traverse_0((Pose,Xs),(Pose,Ys)).

%%
tree_dict(Tree,Dict) :-
	findall(Pair, tree_dict_1(Tree,Pair), Pairs),
	dict_pairs(Dict,_,Pairs).

%%
tree_dict_1(
	[([RefFrame,ChildFrame,Pos,Rot],_)|_],
	ChildFrame-[RefFrame,Pos,Rot]).
tree_dict_1([(_,Xs)|_],Pair) :- tree_dict_1(Xs,Pair).
tree_dict_1([_|Xs],Pair)     :- tree_dict_1(Xs,Pair).
