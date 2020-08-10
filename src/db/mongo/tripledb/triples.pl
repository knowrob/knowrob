:- module(mng_triples,
    [ triple_init/0,
      triple_import/1,
      triple_export/1,
      triple_drop/0,
      triple_tell(r,r,t,+,+),
      triple_ask(r,r,t,+,+,-),
      triple_erase(r,r,t,+,+)
    ]).
/** <module> Triple store backend using mongo DB.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db'),
	[ rdf_meta/1 ]).
:- use_module(library('utility/filesystem'),
	[ path_concat/3 ]).
:- use_module(library('db/subgraph'),
	[ tripledb_get_supgraphs/2]).
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
triple_db(DB,'triples') :- mng_db_name(DB).

%%
%
triple_init :-
	triple_db(DB,Coll),
	% create indices
	forall(
		triple_search_index_(IndexKeys),
		(	append(IndexKeys,['graph'],IndexKeys0),
			append_scope_index_(IndexKeys0,IndexKeys1),
			mng_index_create(DB,Coll,IndexKeys1)
		)
	).

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
triple_ask(QSubject,QProperty,QValue,QScope,FScope,Options) :-
	%% read options 
	option(graph(Graph), Options, user),
	%%
	strip_variable(QSubject,Subject),
	strip_variable(QProperty,Property),
	strip_variable(QValue,ValueQuery),
	%% parse query value
	mng_query_value_(ValueQuery,MngOperator,MngValue,Unit),
	setup_call_cleanup(
		% setup: create a query cursor
		triple_query_cursor_(Subject,Property,
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
triple_erase(Subject,Property,ValueQuery,QScope,Options) :-
	%% read options
	option(graph(Graph), Options, user),
	%% parse query value
	mng_query_value_(ValueQuery,MngOperator,MngValue,Unit),
	%%
	setup_call_cleanup(
		% setup: create a query cursor
		triple_query_cursor_(Subject,Property,
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
		triple_query_cursor_(Subject,Property,
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
triple_query_cursor_(Subject,Property,Operator,MngValue,Unit,Scope,Graph,Cursor) :-
	triple_db(DB,Coll),
	( taxonomical_property(Property,_,_)
	-> ( Key_p='p',  Key_o='o*' )
	;  ( Key_p='p*', Key_o='o' )
	),
	%%
	( atom(Subject)
	-> Query_s=string(Subject)
	;  Query_s=Subject
	),
	( atom(Property)
	-> Query_p=string(Property)
	;  Query_p=Property
	),
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
	),
	mng_cursor_create(DB,Coll,Cursor,[Filter]),
	!.

%%
filter_graph_('*',_) :- !, fail.

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
	;  triple_query_unify_p1(Doc,Property,Options)
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

%%
strip_operator_(    X, =,X) :- var(X).
strip_operator_( =(X), =,X).
strip_operator_(>=(X),>=,X).
strip_operator_(=<(X),=<,X).
strip_operator_( <(X), <,X).
strip_operator_( >(X), >,X).
strip_operator_(    X, =,X).

%%
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
