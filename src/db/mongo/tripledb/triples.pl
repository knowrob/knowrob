:- module(mng_triples,
    [ triple_init/0,
      triple_import/1,
      triple_export/1,
      triple_drop/0,
      triple_tell(r,r,t,+,+),
      triple_ask(r,r,t,+,+,-),
      triple_aggregate(t,+,+,-),
      triple_erase(r,r,t,+,+),
      mng_triple_doc(t,-,t)
    ]).
/** <module> Triple store backend using mongo DB.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db'),
	[ rdf_meta/1 ]).
:- use_module(library('utility/filesystem'),
	[ path_concat/3 ]).
:- use_module(library('db/subgraph')).
:- use_module(library('db/scope')).

:- use_module('../client.pl').
:- use_module('./inference_cache.pl').

:- rdf_meta(taxonomical_property(r,-,-)).

%%
% For "taxonomical" properties, not only the value is stored in the document,
% but as well all its parents (using the key "o*").
%
% TODO: could we auto-expand other properties too?
%       - rdfs:range, rdfs:domain
%       - owl:someValuesFrom, owl:allValuesFrom, owl:onClass, owl:onProperty
%       -- some/all would work, but max cardinality would not!
%
taxonomical_property(P,_,_) :- var(P),!,fail.
taxonomical_property(rdf:type,           subclass_of,    instance_of).
taxonomical_property(rdfs:subClassOf,    subclass_of,    subclass_of).
taxonomical_property(rdfs:subPropertyOf, subproperty_of, subproperty_of).

%%
%
%
triple_db(DB, Name) :- 
	mng_get_db(DB, Name, 'triples').

%%
%
triple_init :-
	triple_db(DB,Coll),
	load_graph_structure,
	% create indices
	(	setting(mng_client:read_only, true)
	->	true
	;	forall(
			triple_search_index_(IndexKeys),
			(	append(IndexKeys,['graph'],IndexKeys0),
				append_scope_index_(IndexKeys0,IndexKeys1),
				mng_index_create(DB,Coll,IndexKeys1)
			)
		)
	).

%%
% This is used to avoid that there are any orphan graphs in case
% of there are some ontologies loaded already into the triple DB
% when KnowRob is started.
%
load_graph_structure :-
	triple_db(DB,Coll),
	mng_distinct_values(DB,Coll,'graph',Names),
	forall(
		member(NameString,Names),
		load_graph_structure1(NameString)
	).

load_graph_structure1(NameString) :-
	string_to_atom(NameString,Name),
	(	Name=user -> true ;
	(	tripledb_add_subgraph(Name,common),
		tripledb_add_subgraph(user,Name)
	)).

%%
% The set of composed indices over triples.
% NOTE: it is not allowed to generate an index over more then one vector field!
%
triple_search_index_(['s']).
triple_search_index_(['p']).
triple_search_index_(['p*']).
triple_search_index_(['o']).
triple_search_index_(['o*']).
triple_search_index_(['s','p']).
triple_search_index_(['s','p*']).
triple_search_index_(['s','o']).
triple_search_index_(['s','o*']).
triple_search_index_(['o','p']).
triple_search_index_(['o','p*']).
triple_search_index_(['p','o*']).
triple_search_index_(['s','o','p']).
triple_search_index_(['s','o','p*']).
triple_search_index_(['s','o*','p']).

%%
% TODO: for some reason it is slower with scope indices?!? (at least tell)
%
append_scope_index_(IndexKeys,IndexKeys).
%append_scope_index_(IndexKeys,IndexKeys0) :-
%	append(IndexKeys, ['scope.time.since', 'scope.time.until'],  IndexKeys0).

%%
%
triple_import(Dir) :-
	triple_db(DB,Coll),
	path_concat(Dir,Coll,Dir0),
	mng_restore(DB,Dir0).

%%
%
triple_export(Dir) :-
	triple_db(_DB,Coll),
	path_concat(Dir,Coll,Dir0),
	mng_export_collection(Coll,Dir0).

%%
%
triple_drop :-
	triple_db(DB,Coll),
	mng_drop(DB,Coll).

%%
%
triple_tell(Subject,Property,ValueQuery,Scope,Options) :-
	%% read options 
	option(graph(Graph),Options,user),
	%% parse query value
	mng_query_value_(ValueQuery,'$eq',MngValue,Unit),
	triple_tell1(Subject,Property,MngValue,Unit,Scope,Graph,Options).

triple_tell1(Subject,Property,MngValue,Unit,Scope,Graph,Options) :-
	% find existing document with *overlapping* scope
	findall(X,
		triple_ask_overlapping_(Subject,Property,MngValue,Unit,Scope,Graph,X),
		[First|Rest]
	),
	!,
	mng_get_dict('scope',First,FirstScope),
	% nothing to do if Scope is a subscope of first
	( subscope_of(Scope,FirstScope)
	-> true
	; (
		% else merge/intersect all overlapping scopes
		( option(update(intersect),Options)
		-> doc_scopes_intersect_(Scope,[First|Rest],MergedScope)
		;  doc_scopes_merge_(Scope,[First|Rest],MergedScope)
		),
		doc_scope_set_(First,MergedScope),
		forall(member(Y,Rest), doc_delete_(Y))
	)).

triple_tell1(S,P,MngValue,Unit,Scope,Graph,Options) :-
	ignore(taxonomical_property(P,HierarchyKey,CacheKey)),
	% create the document
	findall([Key,Val],
		(	( Key='graph', Val=string(Graph) )
		;	( Key='s',     Val=string(S) )
		;	( Key='p',     Val=string(P) )
		;	( var(HierarchyKey),
			  get_supproperties_(P,Properties),
			  Key='p*',    Val=array(Properties)
			)
		;	( Key='o',     Val=MngValue )
		;	( ground(HierarchyKey),
			  fact_value_hierarchy_(HierarchyKey,MngValue,MngHierarchy),
			  Key='o*',    Val=array(MngHierarchy)
			)
		;	( ground(Unit), Key='unit',  Val=string(Unit) )
		;	( get_scope_document_(Scope,Scope_doc),
			  Key='scope', Val=Scope_doc
			)
		),
		Doc
	),
	triple_db(DB,Coll),
	mng_store(DB,Coll,Doc),
	% update other documents
	propagate_assertion_(Doc,CacheKey,S,Graph,Options).

%%
%
mng_triple_doc(triple(S,P,O), Doc, Context) :-
	%% read options
	option(graph(Graph), Context, user),
	option(scope(QScope), Context),
	%%
	strip_variable(S, Subject),
	strip_variable(P, Property),
	strip_variable(O, ValueQuery),
	%%
	% TODO FIXME XXX mng_query_value_/3
	mng_query_value_(Subject,Operator_s,MngSubject),
	mng_query_value_(Property,Operator_p,MngProperty),
	mng_query_value_(ValueQuery,MngOperator,MngValue,Unit),
	triple_query_document_(
		Operator_s, MngSubject,
		Operator_p, MngProperty,
		MngOperator, MngValue, Unit,
		QScope,
		Graph,
		Doc).

%%
%
triple_ask(QSubject,QProperty,QValue,QScope,FScope,Options) :-
	%% read options 
	option(graph(Graph), Options, user),
	%%
	strip_variable(QSubject,Subject),
	strip_variable(QProperty,Property),
	strip_variable(QValue,ValueQuery),
	%%
	% TODO FIXME XXX mng_query_value_/3
	mng_query_value_(Subject,Operator_s,MngSubject),
	mng_query_value_(Property,Operator_p,MngProperty),
	mng_query_value_(ValueQuery,MngOperator,MngValue,Unit),
	setup_call_cleanup(
		% setup: create a query cursor
		triple_query_cursor_(
			Operator_s,MngSubject,
			Operator_p,MngProperty,
			MngOperator,MngValue,Unit,
			QScope,Graph,Cursor
		),
		% call: find matching document
		triple_query_unify_(Cursor,
			QSubject,QProperty,QValue,
			FScope,Options
		),
		% cleanup: destroy cursor again
		mng_cursor_destroy(Cursor)
	).

%%
%
triple_aggregate(
		Triples,
		QScope,FScope,Options) :-
	%% read options 
	option(graph(Graph), Options, user),
	triple_db(DB,Coll),
	%%
	triple_aggregate1(Coll,Triples,QScope,Graph,[],Doc,Vars),
	%%
	setup_call_cleanup(
		% setup: create a query cursor
		mng_cursor_create(DB,Coll,Cursor),
		% call: find matching document
		(	mng_cursor_aggregate(Cursor,['pipeline',array(Doc)]),
			mng_cursor_materialize(Cursor,Result_doc),
			triple_aggregate_unify_(Result_doc,Vars),
			%% handle scope
			% TODO: it would be possible to compute scope intersections in mongo!
			%
			mng_get_dict('v_scope',Result_doc,array(ScopesData)),
			findall(S0,
				(	member(Data0,ScopesData),
					mng_client:mng_doc_value(Data0,S0)
				),
				[FirstScope|RestScopes]
			),
			doc_scopes_intersect_b_(FirstScope,RestScopes,FScope)
		),
		% cleanup: destroy cursor again
		mng_cursor_destroy(Cursor)
	).

%%
triple_aggregate1(_,[],_,_,Vars,[],Vars) :- !.
triple_aggregate1(Coll, [Triple|Xs], QScope, Graph, Vars0, TripleDoc, Vars_n) :-
	read_triple_(Triple,
		Query,
		Operator_s,S,
		Operator_p,P,
		Operator,Value,Unit,
		Options0),
	(	Vars0=[]
	->	Options=[first,property(P)|Options0]
	;	Options=[property(P)|Options0]
	),
	%% 
	read_vars_(Query,S,P,Value,TripleVars),
	%%%%%% recursion
	read_vars_2(Options,TripleVars,Vars1),
	append(Vars0,Vars1,Vars_new),
	list_to_set(Vars_new,Vars2),
	triple_aggregate1(Coll,Xs,QScope,Graph,Vars2,Doc_inner,Vars_n),
	%%%%%% 
	triple_query_document_(
		Operator_s,S,
		Operator_p,P,
		Operator,Value,Unit,
		QScope,Graph,QueryDoc),
	triple_aggregate2(Options,
		Coll,QueryDoc,
		Vars0,TripleVars,
		Doc_inner,
		TripleDoc).

%%	
triple_aggregate2(Options,
		Coll,QueryDoc,
		Vars0,TripleVars,
		Doc_inner, TripleDoc) :-
	findall(Step,
		aggregate_step(Options,Coll,QueryDoc,
				Vars0,TripleVars,
				Step),
		Steps
	),
	append(Steps,Doc_inner,TripleDoc).

%%
aggregate_step(Options,_Coll,QueryDoc,[],_,Step) :-
	%% first step in the pipeline needs special handling:
	%% it uses $match instead of $lookup as first step does not have
	%% any input documents to join with in lookup.
	memberchk(first,Options),
	(	% find matching documents
		Step=['$match', QueryDoc]
	;	% move matching document into field *next* for next steps
		Step=['$set', [
			['next.s', string('$s')],
			['next.p', string('$p')],
			['next.o', string('$o')],
			['next.scope', string('$scope')]
		]]
	;	Step=['$set', ['v_scope', array([string('$scope')]) ]]
	).

aggregate_step(Options,Coll,QueryDoc,Vars0,TripleVars,Step) :-
	%% regular *join* step
	\+ memberchk(first,Options),
    (	aggregate_lookup_(Coll,
			QueryDoc,Options,Vars0,TripleVars,
			Step)
    ;	aggregate_unwind_(Options,Step)
    ).

aggregate_step(Options,_Coll,_QueryDoc,_Vars0,_TripleVars,Step) :-
	%% this step is used to harmonize documents
	\+ memberchk(transitive,Options),
	(	% assign *start* field in case of reflexive property
		(	memberchk(reflexive,Options),
			Step=['$set', ['start', string('$next')]]
		)
	;	% transform next into single-element array
		Step=['$set', ['next', array([string('$next')])]]
	).

aggregate_step(Options,Coll,_QueryDoc,_Vars0,_TripleVars,Step) :-
	%% 
	memberchk(transitive,Options),
	memberchk(property(Property),Options),
	(	Step=['$graphLookup', [
			['from',string(Coll)],
			['startWith',string('$next.o')],
			['connectFromField',string('o')],
			['connectToField',string('s')],
			['as',string('paths')],
			['restrictSearchWithMatch',['p*',string(Property)]]
		]]
	;	Step=['$addFields', ['paths', ['$concatArrays', array([
			string('$paths'),
			array([string('$next')])
		])]]]
	;	Step=['$set', ['start', string('$next')]]
	;	Step=['$set', ['next', string('$paths')]]
	).

aggregate_step(Options,_Coll,_QueryDoc,_Vars0,_TripleVars,Step) :-
	memberchk(reflexive,Options),
	%% FIXME: this creates redundant results for the case of graph queries
	%%        that receive multiple documents with the same subject as input.
	%%        not sure how we can avoid the duplicates as here...
	Step=['$set', ['next', ['$concatArrays',
		array([string('$next'), array([[
			['s',string('$start.s')],
			['p',string('$start.p')],
			['o',string('$start.s')],
			['scope',string('$start.scope')]
		]])])
	]]].

aggregate_step(_Options,_Coll,_QueryDoc,_Vars0,_TripleVars,Step) :-
	Step = ['$unwind',string('$next')].

aggregate_step(Options,_Coll,_QueryDoc,_Vars0,_TripleVars,Step) :-
	%%%%%% $set scopes, $setUnion is used to avoid duplicate scopes
	aggregate_scopes_(Options,Step).

aggregate_step(Options,
		_Coll,_QueryDoc,
		Vars0,TripleVars,
		['$project', ProjectDoc]) :-
	%%%%%% $project
	findall([Pr_Key,Pr_Value],(
		%
		(	Pr_Key='v_scope',
			atom_concat('$',Pr_Key,Pr_Value0),
			Pr_Value=string(Pr_Value0)
		)
		% copy value of var e.g. { 'S': '$S' }
	;	(	member([Pr_Key,_],Vars0),
			atom_concat('$',Pr_Key,Pr_Value0),
			Pr_Value=string(Pr_Value0)
		)
		% set new value of var e.g. { 'S': '$next.s' }
	;	(	member([Pr_Key,_,Field],TripleVars),
			\+ member([Pr_Key,_],Vars0),
			aggregate_project_(Options,Field,Pr_Value)
		)
		% findall special case
	;	(	member(findall(List),Options),
			read_vars_1([[List,'next']],[[Pr_Key,List,Field]]),
			aggregate_project_(Options,Field,Pr_Value)
		)
	), ProjectDoc).

%%
triple_aggregate_unify_(_,[]) :- !.
triple_aggregate_unify_(Doc,[X|Xs]) :-
	triple_aggregate_unify_1(Doc,X),
	triple_aggregate_unify_(Doc,Xs).

triple_aggregate_unify_1(_,['_id',_]) :-
	!.
triple_aggregate_unify_1(Doc,[VarKey,Var]) :-
	mng_get_dict(VarKey,Doc,TypedValue),
	strip_type_(TypedValue,_,Value),
	(	Value='null' ;
		Var=Value
	),
	!.

%%
aggregate_lookup_(Coll,QueryDoc,Options,Vars0,TripleVars,Lookup) :-
	% find all joins with input documents
	findall([Field_j,Value_j],
		(	member([Field_j,_,Value_j],TripleVars),
			member([Field_j,_],Vars0)
		),
		Joins
	),
	% pass input document value to lookup
	findall([Let_key,string(Let_val)],
		(	member([Let_key,_],Joins),
			atom_concat('$',Let_key,Let_val)
		),
		LetDoc
	),
	% perform the join operation (equals the input document value)
	findall(['$eq', array([string(Match_key),string(Match_val)])],
		% { $eq: [ "$s",  "$$R" ] },
		(	member([Join_var,Join_field],Joins),
			atom_concat('$',Join_field,Match_key),
			atom_concat('$$',Join_var,Match_val)
		),
		MatchDoc
	),
	%
	Match=['$match', [['$expr', ['$and', array(MatchDoc)]] | QueryDoc ]],
	(	member(limit(Limit),Options)
	->	Pipeline=[Match,['$limit',int(Limit)]]
	;	Pipeline=[Match]
	),
	% finally compose the lookup document
	Lookup=['$lookup', [
		['from',string(Coll)],
		% create a field "next" with all matching documents
		['as',string('next')],
		% make fields from input document accessible in pipeline
		['let',LetDoc],
		% get matching documents
		['pipeline', array(Pipeline)]
	]].

%%

aggregate_unwind_(Options, _) :-
	memberchk(findall(_),Options),
	!,
	fail.

aggregate_unwind_(Options,
	['$unwind',[
		['path', string('$next')],
		['preserveNullAndEmptyArrays',bool(true)]
	]]) :-
	memberchk(ignore,Options),
	!.

aggregate_unwind_(_,
	['$unwind',string('$next')]) :- !.

%%

aggregate_scopes_(Options,
	['$set',['v_scope',string('$v_scope')]]) :-
	% TODO: handle scope in findall
	memberchk(findall(_),Options),
	!.

aggregate_scopes_(Options,
	['$set',['v_scope',['$cond', array([
		['$not', array([string('$next.scope')]) ],
		string('$v_scope'),
		['$setUnion', array([ string('$v_scope'),
			array([ string('$next.scope') ]) ])]
	])]]]) :-
	memberchk(ignore,Options),
	!.

aggregate_scopes_(_,
	['$set',['v_scope',['$setUnion',
		array([string('$v_scope'), array([string('$next.scope')])])
	]]]) :- !.

%%

aggregate_project_(Options,Field,['$map',[
		['input',string('$next')],
		['in',[
			['s',string('$$this.s')],
			['p',string('$$this.p')],
			['o',string('$$this.o')]
		]]
	]]) :-
	memberchk(findall(_),Options),
	!,
	Field='next'.

aggregate_project_(Options,Field,['$cond',array([
		['$not', array([string(Pr_Value)]) ],
		% REMOVE creates problems in next project step, so rather choose some dump value
		string('null'),
		%string('$$REMOVE'),
		string(Pr_Value)
	])]) :-
	memberchk(ignore,Options),
	!,
	atom_concat('$next.',Field,Pr_Value).

aggregate_project_(_,Field,string(Pr_Value)) :-
	!,
	atom_concat('$next.',Field,Pr_Value).

%%
read_triple_(
		TripleTerm,
		triple(QSubject,QProperty,QValue),
		Operator_s,Subject,
		Operator_p,Property,
		MngOperator,MngValue,Unit,
		Options) :-
	read_triple_1_(TripleTerm,
		triple(QSubject,QProperty,QValue),
		Options
	),
	strip_variable(QSubject,SubjectQuery),
	strip_variable(QProperty,PropertyQuery),
	strip_variable(QValue,ValueQuery),
	mng_query_value_(SubjectQuery,Operator_s,Subject),
	mng_query_value_(PropertyQuery,Operator_p,Property),
	mng_query_value_(ValueQuery,MngOperator,MngValue,Unit).

%%
read_triple_1_(transitive(X),Y,[transitive|Opts]) :-
	!, read_triple_1_(X,Y,Opts).

read_triple_1_(reflexive(X),Y,[reflexive|Opts]) :-
	!, read_triple_1_(X,Y,Opts).

read_triple_1_(ignore(X),Y,[ignore|Opts]) :-
	!, read_triple_1_(X,Y,Opts).

read_triple_1_(once(X),Y,[limit(1)|Opts]) :-
	!, read_triple_1_(X,Y,Opts).

read_triple_1_(limit(X,N),Y,[limit(N)|Opts]) :-
	!, read_triple_1_(X,Y,Opts).

read_triple_1_(findall(X,L),Y,[findall(L)|Opts]) :-
	!, read_triple_1_(X,Y,Opts).

read_triple_1_(X,X,[]) :- !.

%%

read_vars_(triple(QS,QP,QV),S,P,Value,Vars) :-
	once(( (nonvar(QS),QS=(_->S0))     ; strip_type_(S,_,S0) )),
	once(( (nonvar(QP),QP=(_->P0))     ; strip_type_(P,_,P0) )),
	once(( (nonvar(QV),QV=(_->Value0)) ; strip_type_(Value,_,Value0) )),
	read_vars_1([ [S0,'s'], [P0,'p'], [Value0,'o'] ],Vars).
read_vars_1( [],[] ) :- !.
read_vars_1( [[Var,_]|Xs], Ys ) :-
	\+ var(Var),
	!,
	read_vars_1(Xs,Ys).
read_vars_1( [[Var,Field]|Xs],
	         [[Key,Var,Field]|Ys]) :-
	term_to_atom(Var,Var0),
	atom_concat('v',Var0,Key),
	read_vars_1(Xs,Ys).

%%
read_vars_2(Options,_,[[Key,List]]) :-
	member(findall(List),Options),
	!,
	read_vars_1([[List,'next']],[[Key,List,_]]).
read_vars_2(_,[],[]) :- !.
read_vars_2(Options,[[X0,X1,_]|Xs],[[X0,X1]|Ys]) :-
	read_vars_2(Options,Xs,Ys).

%%
%
triple_erase(Subject,Property,ValueQuery,QScope,Options) :-
	%% read options
	option(graph(Graph), Options, user),
	%% parse query value
	mng_query_value_(Subject,Operator_s,MngSubject),
	mng_query_value_(Property,Operator_p,MngProperty),
	mng_query_value_(ValueQuery,MngOperator,MngValue,Unit),
	%%
	setup_call_cleanup(
		% setup: create a query cursor
		triple_query_cursor_(
			Operator_s,MngSubject,
			Operator_p,MngProperty,
			MngOperator,MngValue,Unit,
			QScope,Graph,Cursor
		),
		% call: delete all matching documents
		mng_cursor_erase(Cursor),
		% cleanup: destroy cursor again
		mng_cursor_destroy(Cursor)
	).

%%
triple_ask_overlapping_(Subject,Property,MngValue,Unit,FScope,Graph,OverlappingDoc) :-
	findall(X,scope_overlaps_query(FScope,X),QScopes),
	setup_call_cleanup(
		% setup: create a query cursor
		triple_query_cursor_(
			'$eq',Subject,
			'$eq',Property,
			'$eq',MngValue,Unit,
			QScopes,Graph,Cursor
		),
		% call: find matching document
		mng_cursor_materialize(Cursor,OverlappingDoc),
		% cleanup: destroy cursor again
		mng_cursor_destroy(Cursor)
	).

%%
get_supproperties_(P,SuperProperties) :-
	Options=[graph('*'), include_parents(true)],
	wildcard_scope(QScope),
	findall(string(Sup),
		(	Sup=P
		;	triple_ask(P,rdfs:subPropertyOf,Sup,QScope,_,Options)
		),
		SuperProperties
	).

%%
get_supclasses_(Cls,SuperClasses) :-
	atom(Cls),
	Options=[graph('*'), include_parents(true)],
	wildcard_scope(QScope),
	findall(string(Sup),
		(	Sup=Cls
		;	triple_ask(Cls,rdfs:subClassOf,Sup,QScope,_,Options)
		),
		SuperClasses
	).

		 /*******************************
		 *	   .....              	*
		 *******************************/

%% create a query cursor
triple_query_document_(
		Operator_s, Subject,
		Operator_p, Property,
		Operator,MngValue,Unit,Scope,Graph,Filter) :-
	( taxonomical_property(Property,_,_)
	-> ( Key_p='p',  Key_o='o*' )
	;  ( Key_p='p*', Key_o='o' )
	),
	%%
	( atom(Subject)
	-> Query_s0=string(Subject)
	;  Query_s0=Subject
	),
	(	Operator_s='$eq'
	->	Query_s=Query_s0
	;	Query_s=[Operator_s,Query_s0]
	),
	%%
	(	atom(Property)
	->	Query_p0=string(Property)
	;	Query_p0=Property
	),
	(	Operator_p='$eq'
	->	Query_p=Query_p0
	;	Query_p=[Operator_p,Query_p0]
	),
	%%
	( Operator='$eq'
	-> Query_o=MngValue
	;  Query_o=[Operator,MngValue]
	),
	%%
	findall(X,
		(	( ground(Subject),  X=['s',Query_s] )
		;	( ground(Property), X=[Key_p,Query_p] )
		;	( ground(MngValue), X=[Key_o,Query_o] )
		;	( ground(Unit),     X=['unit',string(Unit)] )
		;	filter_graph_(Graph,X)
		;	filter_scope_(Scope,X)
		),
		Filter
	).

triple_query_cursor_(
		Operator_s, Subject,
		Operator_p, Property,
		Operator,MngValue,Unit,Scope,Graph,Cursor) :-
	triple_query_document_(
		Operator_s, Subject,
		Operator_p, Property,Operator,MngValue,Unit,Scope,Graph,Filter),
	triple_db(DB,Coll),
	mng_cursor_create(DB,Coll,Cursor,[Filter]),
	!.

%%
filter_graph_('*',_) :- !, fail.

% TODO: problems with filtering with this?
filter_graph_('user',_) :- !, fail.

filter_graph_(=(GraphName),['graph',string(GraphName)]) :- !.

filter_graph_(GraphName,['graph',['$in',array(Graphs)]]) :-
	tripledb_get_supgraphs(GraphName,Graphs).

%%
filter_scope_([Scope],Filter) :-
	!, filter_scope_(Scope,Filter).

filter_scope_([First|Rest],['$or',Filters]) :-
	% generate disjunctive query if given a list of scopes
	!,
	findall(['$and',X],
		(	member(Scope,[First|Rest]),
			findall(Y, filter_scope_(Scope,Y), X)
		),
		Filters
	).

filter_scope_(Scope,[Key,Filter]) :-
	get_scope_query_(Scope,Key,Filter).

%%
triple_query_unify_(Cursor,QSubject,QProperty,QValue,FScope,Options) :-
	mng_cursor_materialize(Cursor,Doc),
	triple_query_unify_s(Doc,QSubject),
	triple_query_unify_p(Doc,QProperty,Options),
	triple_query_unify_o(Doc,QValue,Options),
	mng_get_dict('scope',Doc,FScope).

%%
triple_query_unify_s(Doc,QSubject) :-
	get_query_variable(QSubject,Subject),
	( ground(Subject)
	-> true
	;  mng_get_dict('s',Doc,string(Subject))
	),
	!.

%%
triple_query_unify_p(Doc,QProperty,Options) :-
	get_query_variable(QProperty,Property),
	( ground(Property)
	-> true
	;  triple_query_unify_p1(Doc,string(Property),Options)
	).

triple_query_unify_p1(Doc,Property,Options) :-
	((	option(include_parents(true),Options),
		mng_get_dict('p*',Doc,array(Properties)) ) 
	-> member(Property,Properties)
    ;  mng_get_dict('p',Doc,Property)
    ).

%%
triple_query_unify_o(Doc,QValue,Options) :-
	get_query_variable(QValue,ValueQuery),
	( ground(ValueQuery)
	-> true
	;  triple_query_unify_o1(Doc,ValueQuery,Options)
	).

triple_query_unify_o1(Doc,ValueQuery,Options) :-
	triple_query_get_o(Doc,Options,Value),
	once(( mng_get_dict('unit',Doc,string(Unit)); Unit=_ )),
	%%
	once(strip_operator_(ValueQuery,_,Value1)),
	strip_unit_(Value1,Unit,Value2),
	strip_type_(Value2,_,Value3),
	( ground(Value3) -> true ; (
		strip_type_(Value,_,AtomicValue),
		( (nonvar(Value2),Value2=term(_))
		-> term_to_atom(Value3,AtomicValue)
		;  Value3=AtomicValue
		)
	)).

triple_query_get_o(Doc,Options,Value) :-
	% try to read the o/o* field from the mongo document
	(	triple_query_get_o1(Doc,Options,Value)
	*->	true
	% no value could be found, so the document must be corrupted
	;	(	handle_corruption(Doc,field_missing(o)),
			fail
		)
	).

triple_query_get_o1(Doc,Options,Value) :-
	% get value from o* if include_parents option is *true*
	option(include_parents(true),Options),
	!,
	mng_get_dict('o*',Doc,array(Values)),
	member(Value,Values).

triple_query_get_o1(Doc,_Options,Value) :-
	% get value from o if include_parents option is not *true*
	mng_get_dict('o',Doc,Value).

handle_corruption(Doc,Msg) :-
	% read doc id (at least the id should be defined)
	mng_get_dict('_id',Doc,DocID),
	print_message(error, tripledb(document(id(DocID),corruption(Msg)))).

%%
get_query_variable(QValue,Var) :-
	(	var(QValue) -> Var=QValue
	;	QValue=(_->Var) -> true
	;	Var=QValue
	).
		 /*******************************
		 *	   .....              	*
		 *******************************/

%%
mng_query_value_(Query,Operator,Value) :-
	% get operator
	strip_operator_(Query,Operator0,Value),
	operator_mapping_(Operator0,Operator),
	!.

%%
mng_query_value_(Query,Operator,Value,Unit) :-
	% get operator
	strip_operator_(Query,Operator0,Query0),
	operator_mapping_(Operator0,Operator),!,
	% get unit if any
	strip_unit_(Query0,Unit,Query1),!,
	% get the value type
	strip_type_(Query1,Type0,Value0),
	type_mapping_(Type0,MngType),
	(	(Type0=term,compound(Value0))
	->	term_to_atom(Value0,Value1)
	;	Value1=Value0
	),
	Value=..[MngType,Value1],
	!.

% TODO: better use units as replacement of type.
%          I guess all qudt units are double basetype?
strip_unit_(Term,Unit,X) :-
	compound(Term),
	Term=unit(X,Unit),!.
strip_unit_(X,_,X).

%%
% For some properties it is useful to store a hierarchy of values 
% instead of a single one.
%
fact_value_hierarchy_(HierarchyKey,string(Value),List) :-
	(	HierarchyKey=subclass_of    -> get_supclasses_(Value,List)
	;	HierarchyKey=subproperty_of -> get_supproperties_(Value,List)
	;	fail
	),
	!.
fact_value_hierarchy_(_,V0,[V0]).

%%
doc_scopes_merge_(Scope,[],Scope) :- !.
doc_scopes_merge_(Scope0,[First|Rest],MergedScope) :-
	mng_get_dict('scope',First,Scope1),
	scope_merge(Scope0,Scope1,Scope2),
	doc_scopes_merge_(Scope2,Rest,MergedScope).

%%
doc_scopes_intersect_(Scope,[],Scope) :- !.
doc_scopes_intersect_(Scope0,[First|Rest],MergedScope) :-
	mng_get_dict('scope',First,Scope1),
	scope_intersect(Scope0,Scope1,Scope2),
	doc_scopes_intersect_(Scope2,Rest,MergedScope).

doc_scopes_intersect_b_(Scope,[],Scope) :- !.
doc_scopes_intersect_b_(Scope0,[First|Rest],MergedScope) :-
	scope_intersect(Scope0,First,Scope2),
	doc_scopes_intersect_(Scope2,Rest,MergedScope).

%%
doc_scope_set_(Doc,MergedScope) :-
	triple_db(DB,Coll),
	mng_get_dict('_id',Doc,DocID),
	get_scope_document_(MergedScope,ScopeDoc),
	mng_update(DB,Coll,
		['_id', DocID],
		['$set',['scope',ScopeDoc]]).

%%
doc_delete_(Doc) :-
	triple_db(DB,Coll),
	mng_get_dict('_id',Doc,DocID),
	mng_remove(DB,Coll,['_id', DocID]).

%%
get_scope_query_(QScope,Key,Value) :-
	get_dict(ScopeName,QScope,ScopeData),
	get_scope_query2_(ScopeData,SubPath,Value),
	atomic_list_concat([scope,ScopeName,SubPath],'.',Key).

get_scope_query2_(Scope,Path,Value) :-
	is_dict(Scope),!,
	get_dict(Key,Scope,Data),
	get_scope_query2_(Data,SubPath,Value),
	( SubPath=''
	-> Path=Key
	;  atomic_list_concat([Key,SubPath],'.',Path)
	).

get_scope_query2_(Query,'',[Operator,Value]) :-
	mng_query_value_(Query,Operator,Value,_Unit).

%%
:- table(get_scope_document_/2).
get_scope_document_(Dict,List) :- get_scope_document_1(Dict,List).

get_scope_document_1(Dict,List) :-
	is_dict(Dict),
	!,
	findall([K,V],
		(	get_dict(K,Dict,V0),
			get_scope_document_1(V0,V)
		),
		List
	).
get_scope_document_1(X,X).

%%
operator_mapping_('=', '$eq').
operator_mapping_('>=','$gte').
operator_mapping_('=<','$lte').
operator_mapping_('>', '$gt').
operator_mapping_('<', '$lt').
operator_mapping_('in', '$in').
operator_mapping_('nin', '$nin').

%%
strip_operator_(    X, =,X) :- var(X).
strip_operator_( =(X), =,X).
strip_operator_(>=(X),>=,X).
strip_operator_(=<(X),=<,X).
strip_operator_( <(X), <,X).
strip_operator_( >(X), >,X).
strip_operator_(in(X), in,X).
strip_operator_(nin(X),nin,X).
strip_operator_(    X, =,X).

%%
strip_type_(List,array,List) :-
	is_list(List),
	!.
strip_type_(Term,Type,X) :-
	compound(Term),
	!,
	Term=..[Type,X],
	type_mapping_(Type,_).
strip_type_(X,double,X) :- number(X),!.
% FIXME: below makes it impossible to ask for string values true/false
strip_type_(X,bool,X) :- ground(X), (X=true;X=false), !.
% FIXME var(X) always ends in string, but holds takes care of setting type
%                  better do not require type in query!
strip_type_(X,string,X).

%%
strip_variable(X->_,X) :- nonvar(X), !.
strip_variable(X,X) :- !.
  
%%
type_mapping_(float,   double) :- !.
type_mapping_(number,  double) :- !.
type_mapping_(integer, int) :- !.
type_mapping_(long,    int)    :- !.
type_mapping_(short,   int)    :- !.
type_mapping_(byte,    int)    :- !.
type_mapping_(term,    string) :- !.
type_mapping_(X,       X)      :- !.

		 /*******************************
		 *	   PROPAGATION       *
		 *******************************/

%%
propagate_assertion_(Doc,CacheKey,S,Graph,Options) :-
	(	member(['o*',array(Parents)],Doc)
	->	propagate_assertion_1(Parents,CacheKey,S,Graph,Options)
	;	true
	).

propagate_assertion_1(Parents,CacheKey,S,Graph,Options) :-
	% invalidate all inferences when
	% a new axiom was added
	(	option(skip_invalidate(true),Options)
	->	true
	;	inference_cache_invalidate(CacheKey)
	),
	tripledb_get_supgraphs(Graph,Graphs),
	%wildcard_scope(Scope),
	%findall(X,filter_scope1_(Scope,X),ScopeFilter),
	% update documents where S appears in "o*"
	triple_db(DB,Coll),
	mng_update(DB,Coll,
		[ ['graph',['$in',array(Graphs)]],
		  ['o*',string(S)] %|ScopeFilter
		],
		['$addToSet',['o*',['$each',array(Parents)]]]
	).

%%
propagate_deletion_(subclass_of,S,string(_O),Graph) :-
	triple_db(DB,Coll),
	tripledb_get_supgraphs(Graph,Graphs),
	wildcard_scope(QScope),
	% update o* for all triples where o is a subclass of S
	forall(
		triple_ask(X,rdfs:subClassOf,S,QScope,_,[]),
		(	get_supclasses_(X,Parents),
			mng_update(DB,Coll,
				[['graph',['$in',array(Graphs)]],['o',string(X)]],
				['$set',['o*',array(Parents)]])
		)
	),
	!.
propagate_deletion_(subproperty_of,S,string(_O),Graph) :-
	triple_db(DB,Coll),
	tripledb_get_supgraphs(Graph,Graphs),
	wildcard_scope(QScope),
	% update p* for all triples where p is a subproperty of S
	forall(
		triple_ask(X,rdfs:subPropertyOf,S,QScope,_,[]),
		(	get_supproperties_(X,Parents),
			mng_update(DB,Coll,
				[['graph',['$in',array(Graphs)]],['o',string(X)]],
				['$set',['o*',array(Parents)]])
		)
	),
	!.
propagate_deletion_(_,_,_,_).
