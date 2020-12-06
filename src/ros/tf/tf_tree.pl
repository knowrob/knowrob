:- module(tf_tree,
	[ tf_tree_get/2,
	  tf_tree_lookup/3
	]).

:- use_module(library('utility/algebra'),
	[ transform_between/3,
	  transform_multiply/3
	]).
:- use_module(library('db/mongo/client')).
:- use_module('./tf_plugin').

%%
:- dynamic tf_tree_chache/2.

%%
tf_tree_get(Stamp,Dict) :-
	tf_tree_chache_get_(Stamp,Dict),
	!.

tf_tree_get(Stamp,Dict) :-
	% read all required transforms
	lookup_transforms_(Stamp,Transforms),
	% create a tree structure for traversal operation
	tf_tree_create_(Transforms,Tree0),
	% compute world poses
	tf_tree_taverse_(Tree0,Tree1),
	% create dictionary for faster lookup
	tf_tree_dict_(Tree1,Dict),
	% cache tree
	tf_tree_chache_add_(Stamp,Dict).

%%
%
%
tf_tree_lookup(TreeDict,ChildFrame,[RefFrame,Pos,Rot]) :-
	get_dict(ChildFrame,TreeDict,[RefFrame0,Pos0,Rot0]),
	(	RefFrame=RefFrame0
	->	(	Pos=Pos0, Rot=Rot0	)
	;	(	get_dict(RefFrame,TreeDict,[RefFrame0,Pos1,Rot1]),
			transform_between(
				[RefFrame0,ChildFrame,Pos0,Rot0],
				[RefFrame0,RefFrame,Pos1,Rot1],
				[RefFrame,ChildFrame,Pos,Rot])
		)
	),
	!.

%%
tf_tree_chache_get_(Stamp,Tree) :-
	tf_tree_chache(Stamp,Tree).

%%
tf_tree_chache_add_(Stamp,Tree) :-
	retractall(tf_tree_chache(_,_)),
	assertz(tf_tree_chache(Stamp,Tree)).

%%
tf_tree_create_(Transforms,Tree) :-
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
			tf_tree_create_branch_(Frame,Frames_dict,Branch)
		),
		Tree
	).

%%
tf_tree_create_branch_(Frame,Dict,(Pose,Children)) :-
	get_dict(Frame,Dict,[Parent,Pos,Rot,Children0]),
	Pose=[Parent,Frame,Pos,Rot],
	findall(X,
		(	member(ChildFrame,Children0),
			tf_tree_create_branch_(ChildFrame,Dict,X)
		),
		Children).

%%
tf_tree_taverse_([], []) :- !.
tf_tree_taverse_([X|Xs], [Y|Ys]) :-
	tf_tree_taverse_0(X,Y),
	tf_tree_taverse_(Xs,Ys).

%%
tf_tree_taverse_0((Pose,[]), (Pose,[])):- !.
tf_tree_taverse_0((Pose,[X|Xs]), (Pose,[Y|Ys])):-
	tf_tree_taverse_1(Pose,X,Y),
	tf_tree_taverse_0((Pose,Xs), (Pose,Ys)).

%%
tf_tree_taverse_1(Pose_parent, (Pose_child,Xs), (Pose,Ys)) :-
	% get child pose in world frame
	transform_multiply(Pose_parent,Pose_child,Pose),
	% continue for children
	tf_tree_taverse_0((Pose,Xs),(Pose,Ys)).

%%
tf_tree_dict_(Tree,Dict) :-
	findall(Pair,
		tf_tree_dict_1(Tree,Pair),
		Pairs),
	dict_pairs(Dict,_,Pairs).

%%
tf_tree_dict_1(
	[([RefFrame,ChildFrame,Pos,Rot],_)|_],
	ChildFrame-[RefFrame,Pos,Rot]).
tf_tree_dict_1([(_,Xs)|_],Pair) :-
	tf_tree_dict_1(Xs,Pair).
tf_tree_dict_1([_|Xs],Pair) :-
	tf_tree_dict_1(Xs,Pair).

%%
initial_transforms(Transforms) :-
	tf_plugin:tf_db(DB,Coll),
	Query=[
		['$group',['_id',['child_frame_id',string('$child_frame_id')]]],
		['$lookup',[
			['from',string(Coll)],
			['as',string('tf')],
			['let',['frame',string('$_id.child_frame_id')]],
			['pipeline',array([
				['$match',['$expr',['$and',array([
					['$eq',array([string('$child_frame_id'),string('$$frame')])]
				])]]],
				['$sort',[
					['child_frame_id',int(1)],
					['header.stamp',int(-1)]
				]],
				['$limit',int(1)]
			])]
		]],
		['$unwind',string('$tf')],
		['$replaceRoot',['newRoot',string('$tf')]]
	],
	%%
	setup_call_cleanup(
		mng_cursor_create(DB,Coll,Cursor),
		(	mng_cursor_aggregate(Cursor,['pipeline',array(Query)]),
			findall([Ref,Frame,Pos,Rot],
				(	mng_cursor_materialize(Cursor,Doc),
					tf_plugin:tf_mng_doc_pose(Doc,Frame,_,[Ref,Pos,Rot])
				),
				Transforms
			)
		),
		mng_cursor_destroy(Cursor)
	).

%%
lookup_transforms_(Stamp,Transforms) :-
	tf_plugin:tf_db(DB,Coll),
	Query=[
		% get distinct frames
		/*
		['$match',['$expr',
			['$lt',array([string('$header.stamp'),time(Stamp)])]
		]],
		['$sort',[
			['child_frame_id',int(1)],
			['header.stamp',int(1)]
		]],
		['$group',[
			['_id',string('$child_frame_id')],
			['child_frame_id',['$first',string('$child_frame_id')]],
			['header',['$first',string('$header')]],
			['transform',['$first',string('$transform')]]
		]]
		*/
		['$group',['_id',['child_frame_id',string('$child_frame_id')]]],
		['$lookup',[
			['from',string(Coll)],
			['as',string('tf')],
			['let',['frame',string('$_id.child_frame_id')]],
			['pipeline',array([
				['$match',['$expr',['$and',array([
					['$eq',array([string('$child_frame_id'),string('$$frame')])],
					['$lt',array([string('$header.stamp'),time(Stamp)])]
				])]]],
				['$sort',[
					['child_frame_id',int(1)],
					['header.stamp',int(-1)]
				]],
				['$limit',int(1)]
			])]
		]],
		['$unwind',string('$tf')],
		['$replaceRoot',['newRoot',string('$tf')]]
	],
	%%
	setup_call_cleanup(
		mng_cursor_create(DB,Coll,Cursor),
		(	mng_cursor_aggregate(Cursor,['pipeline',array(Query)]),
			findall([Ref,Frame,Pos,Rot],
				(	mng_cursor_materialize(Cursor,Doc),
					tf_plugin:tf_mng_doc_pose(Doc,Frame,_,[Ref,Pos,Rot])
				),
				Transforms
			)
		),
		mng_cursor_destroy(Cursor)
	).

/*
%%
lookup_transforms_(Frames,QS,Transforms) :-
	%%
	time_scope_data(QS,[_QSince,QUntil]),
	Stamp is QUntil,
	%%
	lookup_transforms_0(Frames,Frames_q0,Frames_q1,Facets_q),
	Query=[
		['$match',['$or',array(Frames_q0)]],
		% TODO: must be tested (better after sort?? or before prev match?)
		['$match',['$lt',['header.stamp',time(Stamp)]]],
		['$sort',[
			["child_frame_id",int(1)],
			["header.stamp",int(1)]
		]],
		['$facet',Facets_q],
		['$project',['tf',['$setUnion',array(Frames_q1)]]],
		['$unwind',string('tf')],
		['$replaceRoot',['newRoot',string('$tf')]]
	],
	%%
	tf_db(DB,Coll),
	setup_call_cleanup(
		mng_cursor_create(DB,Coll,Cursor),
		(	mng_cursor_aggregate(Cursor,['pipeline',array(Query)]),
			findall([Frame,Ref,Pos,Rot],
				(	mng_cursor_materialize(Cursor,Doc),
					tf_mng_doc_pose(Doc,Frame,_,[Ref,Pos,Rot])
				),
				Transforms
			)
		),
		mng_cursor_destroy(Cursor)
	).

%%
lookup_transforms_0([],[],[],[]) :- !.
lookup_transforms_0([X|Xs],
		[['child_frame_id',string(X)]|As],
		[string(X0)|Bs],
		[Facet|Cs]) :-
	atom_concat('$',X,X0),
	Facet=[X,array(
		['$match',['child_frame_id',string(X)]],
		['$limit',int(1)]
	)],
	lookup_transforms_0(Xs,As,Bs,Cs).
*/

