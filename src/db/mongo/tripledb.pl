:- module(mng_tripledb,
    [ implements('db/itripledb')
    ]).
/** <module> Triple store backend using mongo DB.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db'),      [ rdf_meta/1 ]).
:- use_module(library('utility/filesystem'), [ path_concat/3 ]).
:- use_module(library('db/subgraph'),        [ tripledb_get_supgraphs/2]).
:- use_module(library('db/scope')).

:- use_module('./client.pl').

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
taxonomical_property(rdf:type,           subclass_of,    instance_of).
taxonomical_property(rdfs:subClassOf,    subclass_of,    subclass_of).
taxonomical_property(rdfs:subPropertyOf, subproperty_of, subproperty_of).

%%
%
%
tripledb(DB,'tripledb','inferred') :- mng_db_name(DB).

%% 
% @implements 'db/itripledb'
%
tripledb_init :-
  tripledb(DB,TriplesColl,InferredColl),
  %%
  forall(
    tripledb_search_index_(IndexKeys), (
    append(IndexKeys,['graph'],IndexKeys0),
    append_scope_index_(IndexKeys0,IndexKeys1),
    mng_index_create(DB,TriplesColl,IndexKeys1)
  )),
  mng_index_create(DB,InferredColl,['query']),
  mng_index_create(DB,InferredColl,['predicate']).

%%
% The set of composed indices over triples.
% NOTE: it is not allowed to generate an index over more then one vector field!
%
tripledb_search_index_(['s']).
tripledb_search_index_(['p']).
tripledb_search_index_(['p*']).
tripledb_search_index_(['o']).
tripledb_search_index_(['o*']).
tripledb_search_index_(['s','p']).
tripledb_search_index_(['s','p*']).
tripledb_search_index_(['s','o']).
tripledb_search_index_(['s','o*']).
tripledb_search_index_(['o','p']).
tripledb_search_index_(['o','p*']).
tripledb_search_index_(['p','o*']).
tripledb_search_index_(['s','o','p']).
tripledb_search_index_(['s','o','p*']).
tripledb_search_index_(['s','o*','p']).

%%
% TODO: for some reason it is slower with scope indices?!? (at least tell)
%
append_scope_index_(IndexKeys,IndexKeys).
%append_scope_index_(IndexKeys,IndexKeys0).
  %append(IndexKeys, %['scope.time.since', 'scope.time.until'],  %IndexKeys0),

%% 
% @implements 'db/itripledb'
%
tripledb_import(Dir) :-
  tripledb(DB,TriplesColl,InferredColl),
  forall(
    ( path_concat(Dir,TriplesColl,Dir0) ;
      path_concat(Dir,InferredColl,Dir0) ),
    ( mng_restore(DB,Dir0) )).

%% 
% @implements 'db/itripledb'
%
tripledb_export(Dir) :-
  tripledb(_DB,TriplesColl,InferredColl),
  path_concat(Dir,TriplesColl,Dir0),
  path_concat(Dir,InferredColl,Dir1),
  mng_export_collection(TriplesColl,Dir0),
  mng_export_collection(InferredColl,Dir1).

%% 
% @implements 'db/itripledb'
%
tripledb_whipe :-
  tripledb(DB,TriplesColl,InferredColl),
  mng_drop(DB,TriplesColl),
  mng_drop(DB,InferredColl).

%% 
%
tripledb_bulk_tell(Triples,Scope,Options) :-
  % TODO: support bulk insert.
  %         - needs support for bulk operations (mongoc_collection_create_bulk_operation)
  %         - make sure no duplicates are added
  %         - still need to propagate subclass/subproperty assertions!
  forall(
    member(rdf(S,P,O),Triples),
    tripledb_tell(S,P,O,Scope,Options)
  ).

%% 
% @implements 'db/itripledb'
%
tripledb_tell(Subject,Property,ValueQuery,Scope,Options) :-
  %% read options 
  option(graph(Graph),Options,user),
  %% parse query value
  mng_query_value_(ValueQuery,'$eq',MngValue,Unit),
  tripledb_tell1(Subject,Property,MngValue,Unit,Scope,Graph,Options).

tripledb_tell1(Subject,Property,MngValue,Unit,Scope,Graph,Options) :-
  % find existing document with *overlapping* scope
  findall(X,
    tripledb_ask_overlapping_(Subject,Property,MngValue,Unit,Scope,Graph,X),
    [First|Rest]),!,
  mng_get_dict('scope',First,FirstScope),
  % nothing to do if Scope is a subscope of first
  ( subscope_of(Scope,FirstScope) -> true ; (
    % else merge/intersect all overlapping scopes
    ( option(update(intersect),Options)
    -> doc_scopes_intersect_(Scope,[First|Rest],MergedScope)
    ;  doc_scopes_merge_(Scope,[First|Rest],MergedScope)
    ),
    doc_scope_set_(First,MergedScope),
    forall(member(Y,Rest), doc_delete_(Y))
  )).

tripledb_tell1(S,P,MngValue,Unit,Scope,Graph,Options) :-
  ignore( taxonomical_property(P,HierarchyKey,CacheKey) ),
  triple_doc1(rdf(S,P,MngValue,Unit,HierarchyKey),Scope,Graph,Doc),
  tripledb(DB,TriplesColl,_),
  mng_store(DB,TriplesColl,Doc),
  % update other documents
  propagate_assertion_(HierarchyKey,CacheKey,S,MngValue,Graph,Options).

%%
triple_doc(rdf(S,P,ValueQuery),Scope,Graph,Doc) :-
  ignore( taxonomical_property(P,HierarchyKey,_) ),
  mng_query_value_(ValueQuery,'$eq',MngValue,Unit),
  triple_doc1(rdf(S,P,MngValue,Unit,HierarchyKey),Scope,Graph,Doc).

triple_doc1(rdf(S,P,MngValue,Unit,HierarchyKey),Scope,Graph,Doc) :-
  findall([Key,Val], (
    ( Key='graph', Val=string(Graph) );
    ( Key='s',     Val=string(S) );
    ( Key='p',     Val=string(P) );
    ( var(HierarchyKey),
      get_supproperties_(P,Properties),
      Key='p*',    Val=array(Properties) );
    ( Key='o',     Val=MngValue );
    ( ground(HierarchyKey),
      fact_value_hierarchy_(HierarchyKey,MngValue,MngHierarchy),
      Key='o*',    Val=array(MngHierarchy) );
    ( ground(Unit),
      Key='unit',  Val=string(Unit) );
    ( get_scope_document_(Scope,Scope_doc),
      Key='scope', Val=Scope_doc )
  ), Doc).

%% 
% @implements 'db/itripledb'
%
tripledb_ask(Subject,Property,ValueQuery,QScope,FScope,Options) :-
  %% read options 
  option(graph(Graph), Options, user),
  %% parse query value
  mng_query_value_(ValueQuery,MngOperator,MngValue,Unit),
  setup_call_cleanup(
    % setup: create a query cursor
    triple_query_cursor_(Subject,Property,
            MngOperator,MngValue,Unit,
            QScope,Graph,Cursor),
    % call: find matching document
    triple_query_unify_(Cursor,Subject,Property,
            ValueQuery,FScope,Options),
    % cleanup: destroy cursor again
    mng_cursor_destroy(Cursor)
  ).

%%
tripledb_ask_overlapping_(Subject,Property,MngValue,Unit,FScope,Graph,OverlappingDoc) :-
  findall(X,scope_overlaps_query(FScope,X),QScopes),
  setup_call_cleanup(
    % setup: create a query cursor
    triple_query_cursor_(Subject,Property,
            '$eq',MngValue,Unit,
            QScopes,Graph,Cursor),
    % call: find matching document
    mng_cursor_materialize(Cursor,OverlappingDoc),
    % cleanup: destroy cursor again
    mng_cursor_destroy(Cursor)
  ).

%% 
% @implements 'db/itripledb'
%
tripledb_forget(Subject,Property,ValueQuery,Scope,Options) :-
  %% read options 
  option(graph(Graph), Options, user),
  %% parse query value
  mng_query_value_(ValueQuery,Operator,MngValue,Unit),
  %%
  triple_query_cursor_(Subject,Property,
        Operator,MngValue,Unit,
        Scope,Graph,Cursor),
  mng_cursor_erase(Cursor),
  mng_cursor_destroy(Cursor),
  %%
  ignore( taxonomical_property(Property,HierarchyKey,_) ),
  propagate_deletion_(HierarchyKey,Subject,MngValue,Graph).

%%
get_supproperties_(P,SuperProperties) :-
  %Options=[graph(tbox), include_parents(true)],
  Options=[include_parents(true)],
  wildcard_scope(QScope),
  findall(string(Sup),
    (Sup=P ; tripledb_ask(P,rdfs:subPropertyOf,Sup,QScope,_,Options)),
    SuperProperties).

%%
get_supclasses_(Cls,SuperClasses) :-
  %Options=[graph(tbox), include_parents(true)],
  Options=[include_parents(true)],
  wildcard_scope(QScope),
  findall(string(Sup),
    (Sup=Cls ; tripledb_ask(Cls,rdfs:subClassOf,Sup,QScope,_,Options)),
    SuperClasses).

		 /*******************************
		 *	   .....              	*
		 *******************************/

%% create a query cursor
triple_query_cursor_(Subject,Property,Operator,MngValue,Unit,Scope,Graph,Cursor) :-
  tripledb(DB,TriplesColl,_),
  tripledb_get_supgraphs(Graph,Graphs),
  ( taxonomical_property(Property,_,_) ->
    (Key_p='p',  Key_o='o*') ;
    (Key_p='p*', Key_o='o')
  ),
  mng_cursor_create(DB,TriplesColl,Cursor),
  ( \+ground(Subject)  ; mng_cursor_filter(Cursor,['s',string(Subject)]) ),
  ( \+ground(MngValue) ; mng_cursor_filter(Cursor,[Key_o,[Operator,MngValue]]) ),
  ( \+ground(Property) ; mng_cursor_filter(Cursor,[Key_p,string(Property)]) ),
  ( \+ground(Unit)     ; mng_cursor_filter(Cursor,['unit',string(Unit)]) ),
  mng_cursor_filter(Cursor,['graph',['$in',array(Graphs)]]),
  filter_scope_(Cursor,Scope),
  !.

filter_scope_(Cursor,[Scope]) :-
  !, filter_scope_(Cursor,Scope).

filter_scope_(Cursor,Scopes) :-
  % generate disjunctive query if given a list of scopes
  is_list(Scopes),!,
  findall(['$and',X], (
    member(Scope,Scopes),
    findall(Y, filter_scope1_(Scope,Y), X)),
    Filters),
  mng_cursor_filter(Cursor,['$or',Filters]).

filter_scope_(Cursor,Scope) :-
  forall(
    filter_scope1_(Scope,Filter),
    mng_cursor_filter(Cursor,Filter)).

filter_scope1_(Scope,[Key,Filter]) :-
  get_scope_query_(Scope,Key,Filter).

%%
triple_query_unify_(Cursor,Subject,Property,ValueQuery,FScope,Options) :-
  mng_cursor_materialize(Cursor,Doc),
  ( ground(Subject)    -> true ; mng_get_dict('s',Doc,string(Subject)) ),
  ( ground(Property)   -> true ; triple_property_unify1_(Doc,Property,Options) ),
  ( ground(ValueQuery) -> true ; triple_query_unify1_(Doc,ValueQuery,Options) ),
  % get the fact scope
  mng_get_dict('scope',Doc,FScope).

triple_property_unify1_(Doc,Property,Options) :-
  ( ( option(include_parents(true),Options),
      mng_get_dict('p*',Doc,array(Properties)) ) ->
    ( member(Property,Properties) );
    ( mng_get_dict('p',Doc,Property) )
  ).

triple_query_unify1_(Doc,ValueQuery,Options) :-
  ( ( option(include_parents(true),Options),
      mng_get_dict('o*',Doc,array(Values)) ) ->
    ( member(Value,Values) );
    ( mng_get_dict('o',Doc,Value) )
  ),
  once(( mng_get_dict('unit',Doc,string(Unit)); Unit=_ )),
  %%
  strip_operator_(ValueQuery,_,Value1),
  strip_unit_(Value1,Unit,Value2),
  strip_type_(Value2,_,Value3),
  ( ground(Value3)
  -> true
  ;  strip_type_(Value,_,Value3)
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
  % finally get value
  strip_type_(Query1,Type0,Value0),
  type_mapping_(Type0,MngType),
  Value=..[MngType,Value0],
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
  ( HierarchyKey=subclass_of    -> get_supclasses_(Value,List);
    HierarchyKey=subproperty_of -> get_supproperties_(Value,List);
    fail
  ),!.
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
  tripledb(DB,TriplesColl,_),
  mng_get_dict('_id',Doc,DocID),
  get_scope_document_(MergedScope,ScopeDoc),
  mng_update(DB,TriplesColl,
    ['_id', DocID],
    ['$set',['scope',ScopeDoc]]).

%%
doc_delete_(Doc) :-
  tripledb(DB,TriplesColl,_),
  mng_get_dict('_id',Doc,DocID),
  mng_remove(DB,TriplesColl,['_id', DocID]).

%%
get_scope_query_(QScope,Key,Value) :-
  get_dict(ScopeName,QScope,ScopeData),
  get_scope_query2_(ScopeData,SubPath,Value),
  atomic_list_concat([scope,ScopeName,SubPath],'.',Key).

get_scope_query2_(Scope,Path,Value) :-
  is_dict(Scope),!,
  get_dict(Key,Scope,Data),
  get_scope_query2_(Data,SubPath,Value),
  ( SubPath='' -> Path=Key;
    atomic_list_concat([Key,SubPath],'.',Path)
  ).

get_scope_query2_(Query,'',[Operator,Value]) :-
  mng_query_value_(Query,Operator,Value,_Unit).

%%
get_scope_document_(Dict,List) :-
  is_dict(Dict),!,
  findall([K,V], (
    get_dict(K,Dict,V0),
    get_scope_document_(V0,V)
  ), List).
get_scope_document_(X,X).

%%
operator_mapping_('=', '$eq').
operator_mapping_('>=','$gte').
operator_mapping_('=<','$lte').
operator_mapping_('>', '$gt').
operator_mapping_('<', '$lt').

%%
strip_operator_(    X, =,X) :- var(X),!.
strip_operator_( =(X), =,X) :- !.
strip_operator_(>=(X),>=,X) :- !.
strip_operator_(=<(X),=<,X) :- !.
strip_operator_( <(X), <,X) :- !.
strip_operator_( >(X), >,X) :- !.
strip_operator_(    X, =,X) :- !.

%%
strip_type_(Term,Type,X) :-
  compound(Term),!,
  Term=..[Type,X],
  type_mapping_(Type,_).
strip_type_(X,double,X) :- number(X),!.
% FIXME: below makes it impossible to ask for string values true/false
strip_type_(X,bool,X) :- ground(X), (X=true;X=false), !.
% FIXME var(X) always ends in string, but holds takes care of setting type
%                  better do not require type in query!
strip_type_(X,string,X).
  
%%
type_mapping_(float,  double).
type_mapping_(double, double).
type_mapping_(number, double).
type_mapping_(long,   int).
type_mapping_(int,    int).
type_mapping_(short,  int).
type_mapping_(byte,   int).
type_mapping_(string, string).
type_mapping_(bool,   bool).

		 /*******************************
		 *	   PROPAGATION       *
		 *******************************/

%%
propagate_assertion_(HierarchyKey,CacheKey,S,string(O),Graph,Options) :-
  ground(HierarchyKey),!,
  % invalidate all inferences when
  % a new axiom was added
  ( option(skip_invalidate(true),Options) ->
    ( true ) ;
    ( tripledb_cache_invalidate(CacheKey) )
  ),
  % find parents
  ( HierarchyKey=subclass_of    -> get_supclasses_(O,Parents);
    HierarchyKey=subproperty_of -> get_supproperties_(O,Parents);
    fail
  ),
  tripledb_get_supgraphs(Graph,Graphs),
  %wildcard_scope(Scope),
  %findall(X,filter_scope1_(Scope,X),ScopeFilter),
  % update documents where S appears in "o*"
  tripledb(DB,TriplesColl,_),
  mng_update(DB,TriplesColl,
    [ ['graph',['$in',array(Graphs)]],
      ['o*',string(S)] %|ScopeFilter
	],
    ['$addToSet',['o*',['$each',array(Parents)]]]
  ).
propagate_assertion_(_,_,_,_,_,_).

%%
propagate_deletion_(subclass_of,S,string(_O),Graph) :-
  tripledb(DB,TriplesColl,_),
  tripledb_get_supgraphs(Graph,Graphs),
  wildcard_scope(QScope),
  % update o* for all triples where o is a subclass of S
  forall(
    tripledb_ask(X,rdfs:subClassOf,S,QScope,_,[]),
    ( get_supclasses_(X,Parents),
      mng_update(DB,TriplesColl,
        [['graph',['$in',array(Graphs)]],['o',string(X)]],
        ['$set',['o*',array(Parents)]])
    )
  ),
  !.
propagate_deletion_(subproperty_of,S,string(_O),Graph) :-
  tripledb(DB,TriplesColl,_),
  tripledb_get_supgraphs(Graph,Graphs),
  wildcard_scope(QScope),
  % update p* for all triples where p is a subproperty of S
  forall(
    tripledb_ask(X,rdfs:subPropertyOf,S,QScope,_,[]),
    ( get_supproperties_(X,Parents),
      mng_update(DB,TriplesColl,
        [['graph',['$in',array(Graphs)]],['o',string(X)]],
        ['$set',['o*',array(Parents)]])
    )
  ),
  !.
propagate_deletion_(_,_,_,_).

		 /*******************************
		 *	   CACHING       *
		 *******************************/

:- dynamic tripledb_cache_empty/1.

%% 
% @implements 'db/itripledb'
%
%
tripledb_cache_get(Predicate,_,[]) :-
  tripledb_cache_empty(Predicate),!.

tripledb_cache_get(_Predicate,Query,Modules) :-
  tripledb(DB,_,InferredColl),
  mng_cursor_create(DB,InferredColl,Cursor),
  mng_cursor_filter(Cursor,['query',string(Query)]),
  findall(M, (
    mng_cursor_materialize(Cursor,Doc),
    mng_get_dict('module',Doc,string(M))
  ), Modules),
  mng_cursor_destroy(Cursor).

%% 
% @implements 'db/itripledb'
%
tripledb_cache_add(Predicate,Query,Module) :-
  tripledb(DB,_,InferredColl),
  mng_store(DB,InferredColl,[
        [query,     string(Query)],
        [module,    string(Module)],
        [predicate, string(Predicate)]
  ]),
  retractall(tripledb_cache_empty(Predicate)).

%% 
% @implements 'db/itripledb'
%
tripledb_cache_invalidate(Predicate) :-
  tripledb_cache_empty(Predicate),!.

tripledb_cache_invalidate(Predicate) :-
  tripledb(DB,_,InferredColl),
  mng_cursor_create(DB,InferredColl,Cursor),
  mng_cursor_filter(Cursor,['predicate',string(Predicate)]),
  mng_cursor_erase(Cursor),
  mng_cursor_destroy(Cursor),
  assertz(tripledb_cache_empty(Predicate)).
