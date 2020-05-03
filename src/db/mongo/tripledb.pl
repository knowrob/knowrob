:- module(mng_tripledb,
    [ implements('db/itripledb')
    ]).
/** <module> Triple store backend using mongo DB.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db'), 
    [ rdf_meta/1
    ]).
:- use_module(library('utility/filesystem'), 
    [ path_concat/2
    ]).

:- use_module('./client.pl').

:- dynamic subgraph_of_/2.

:- rdf_meta propagate_assertion_(r,+,+).
:- rdf_meta propagate_deletion_(r,+,+).
:- rdf_meta taxonomical_property(r,?).

%% TODO this should not be here.
%        also make it configurable.
% NOTE: some/all would work, but max cardinality would not!
%taxonomical_property(owl:someValuesFrom, class).
%taxonomical_property(owl:allValuesFrom,  class).
%taxonomical_property(owl:onClass,        class).
%taxonomical_property(owl:onProperty,     property).
taxonomical_property(rdf:type,           class).
taxonomical_property(rdfs:subClassOf,    class).
taxonomical_property(rdfs:subPropertyOf, property).
% TODO: what about range/domain???
%taxonomical_property(rdfs:range, class).
%taxonomical_property(rdfs:domain, class).

%%
tripledb(DB,'tripledb','inferred') :- mng_db_name(DB).

%%
% @implements 'db/itripledb'
%
tripledb_subgraph_of(Sub,Sup) :-
  assert_subgraph_of_(Sub,Sup),
  % transitivity
  forall(
    ( subgraph_of_(Sup,Sup0),
    ( X=Sub ; subgraph_of_(X,Sub) )),
    ( assert_subgraph_of_(X,Sup0) )
  ).

%%
assert_subgraph_of_(Sub,Sup) :-
  subgraph_of_(Sub,Sup),!.
assert_subgraph_of_(Sub,Sup) :-
  assertz(subgraph_of_(Sub,Sup)).

%%
get_supgraphs_(G,_) :-
  var(G),!.
get_supgraphs_(G,Graphs) :-
  findall(string(X), (
    X=G; subgraph_of_(G,X)
  ),Graphs).

%% 
% @implements 'db/itripledb'
%
tripledb_init :-
  tripledb(DB,TriplesColl,InferredColl),
  mng_index_create(DB,TriplesColl,
    [ 'query',
      'predicate'
    ]).
  %%
  % TODO: look deeper into creating indices
  % TODO: scopes should define their indices
  mng_index_create(DB,InferredColl,
    [ 's',
      'p*',
      'o*',
      'graph',
      'scope.time.since',
      'scope.time.until'
    ]).

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
% @implements 'db/itripledb'
%
tripledb_load_rdf(RDF_Data,Scope,Graph) :-
  % TODO: bulk insert instead
  % TODO: support triple stream
  forall(
    member(rdf(S,P,O),RDF_Data),
    tripledb_tell(S,P,O,Scope,Graph)
  ).

%% 
% @implements 'db/itripledb'
%
tripledb_tell(Subject,Property,ValueQuery,Scope,Graph) :-
  mng_query_value_(ValueQuery,=,MngValue,Unit),
  tripledb_tell1(Subject,Property,MngValue,Unit,Scope,Graph).

tripledb_tell1(Subject,Property,MngValue,Unit,Scope,_Graph) :-
  % find existing document
  tripledb_document1(Subject,Property,MngValue,Unit,Doc),!,
  Query=[ ['s',Subject],
          ['p',Property],
          ['o',MngValue] ],
  % handle unit
  ( var(Unit) ->
    Query0=Query ;
    Query0=[['unit',Unit]|Query] ),
  % update scope
  mng_get_dict('scope',Doc,array(DocScopes)),
  ( update_scope_(DocScopes,Scope,NewScopes) ->
    ( mng_update(DB,Coll,Query0,
        [ 'scope',['$eq',array(NewScopes)] ]) );
    % nothing changed
    ( true )
  ).

tripledb_tell1(Subject,Property,MngValue,Unit,Scope,Graph) :-
  % find all super properties of Property
  findall(string(Y), tripledb_subproperty_of(Property,Y), Properties),
  % create a new document
  findall(Key-Val, (
    ( Key='graph', Val=string(Graph) );
    ( Key='s',     Val=string(Subject) );
    ( Key='p',     Val=string(Property) );
    ( Key='p*',    Val=array(Properties) );
    ( Key='o',     Val=MngValue );
    ( fact_value_hierarchy_(Property,MngValue,MngHierarchy),
      Key='o*',    Val=array(MngHierarchy) );
    ( ground(Unit),
      Key='unit',  Val=string(Unit) );
    ( get_scope_document_(Scope,Scope_doc),
      Key='scope', Val=array([Scope_doc]) )
  ), Pairs),
  dict_pairs(Dict,_,Pairs),
  tripledb(DB,TriplesColl,_),
  mng_store(DB,TriplesColl,Dict),
  % update other documents
  propagate_assertion_(Property,Subject,MngValue).

%% 
% @implements 'db/itripledb'
%
tripledb_ask(Subject,Property,ValueQuery,QScope,FScope,Graph) :-
  mng_query_value_(ValueQuery,Operator,MngValue,Unit),
  setup_call_cleanup(
    % setup: create a query cursor
    triple_query_cursor_(Subject,Property,
            MngOperator,MngValue,Unit,
            QScope,Graph,Cursor),
    % call: find matching document
    triple_query_unify_(Cursor,Subject,Property,
            ValueQuery,QScope,FScope,Graph),
    % cleanup: destroy cursor again
    mng_cursor_destroy(Cursor)
  ).

%%
tripledb_document(Subject,Property,ValueQuery,Doc) :-
  mng_query_value_(ValueQuery,=,Value,Unit),
  tripledb_document1(Subject,Property,Value,Unit,Doc).

tripledb_document1(Subject,Property,Value,Unit,Doc) :-
  tripledb(DB,TriplesColl,_),
  mng_cursor_create(DB,TriplesColl,Cursor),
  mng_cursor_filter(Cursor,['s',string(Subject)]),
  mng_cursor_filter(Cursor,['p',string(Property)]),
  mng_cursor_filter(Cursor,['o',Value]),
  % TODO: think about if we can use * here
  %mng_cursor_filter(Cursor,['p*',string(Property)]),
  %mng_cursor_filter(Cursor,['o*',Value]),
  ( var(Unit); mng_cursor_filter(Cursor,['unit',string(Unit)]) ),
  %%
  mng_cursor_materialize(Cursor,Doc),
  mng_cursor_destroy(Cursor),
  !.

%% 
% @implements 'db/itripledb'
%
tripledb_forget(Subject,Property,ValueQuery,Scope,Graph) :-
  mng_query_value_(ValueQuery,Operator,MngValue,Unit),
  triple_query_cursor_(Subject,Property,
        Operator,MngValue,Unit,
        Scope,Graph,Cursor),
  mng_cursor_erase(Cursor),
  mng_cursor_destroy(Cursor),
  propagate_deletion_(Property,Subject,MngValue).

		 /*******************************
		 *	   .....              	*
		 *******************************/

%% create a query cursor
triple_query_cursor_(Subject,Property,Operator,MngValue,Unit,Scope,Graph,Cursor) :-
  tripledb(DB,TriplesColl,_),
  get_supgraphs_(Graph,Graphs),
  mng_cursor_create(DB,TriplesColl,Cursor),
  ( \+ground(Subject)  ; mng_cursor_filter(Cursor,['s',Subject]) ),
  ( \+ground(Property) ; mng_cursor_filter(Cursor,['p*',Property]) ),
  ( \+ground(MngValue) ; mng_cursor_filter(Cursor,['o*',[Operator,MngValue]]) ),
  ( \+ground(Unit)     ; mng_cursor_filter(Cursor,['unit',Unit]) ),
  ( \+ground(Graphs)   ; mng_cursor_filter(Cursor,['graph',['$in',array(Graphs)]]) ),
  % filter scope
  forall(
    ( get_scope_query_(Scope,Scope_Key,Scope_Query),
      mng_query_value_(Scope_Query,Scope_Operator,Scope_Value,_) ),
    % TODO: here we also match all documents without that scope.
    %        this is to match time scopes where until is not known,
    %        expecting that this means it is still ongoing.
    %        - better distinguish between ongoing, and unknown!
    %        - better use $exists=0 only for selected keys? e.g. time.until
    ( mng_cursor_filter(Cursor,
        [ Scope_Key,['$or',[
            ['$exists',bool(0)],
            [Scope_Operator,Scope_Value]
        ]]] ))
  ).

%%
triple_query_unify_(Cursor,Subject,Property,ValueQuery,QScope,FScope,Graph) :-
  MngValue=..[_,Value],
  mng_cursor_materialize(Cursor,Doc),
  ( ground(Graph)      -> true ; mng_get_dict('graph',Doc,string(Graph)) ),
  ( ground(Subject)    -> true ; mng_get_dict('s',Doc,string(Subject)) ),
  ( ground(Property)   -> true ; mng_get_dict('p',Doc,string(Property)) ),
  ( ground(ValueQuery) -> true ; triple_query_unify1_(Doc,ValueQuery) ),
  % get the fact scope
  ( mng_get_dict('scope',Doc,array(FScopes)) ->
    % only yield fact scopes that satisfy the query scope
    ( member(FScope,FScopes), scope_satisfies(FScope,QScope) );
    ( universal_scope(FScope) )
  ).

triple_query_unify1_(Doc,ValueQuery) :-
  mng_get_dict('o',Doc,Value),
  strip_type_(Value,_,Value0),
  ( mng_get_dict('unit',Doc,string(Unit)) -> 
    true ;
    Unit=_ ),
  triple_query_unify2_(Value0,Unit,ValueQuery).

triple_query_unify2_(Value,Unit,ValueQuery) :-
  var(ValueQuery),!,
  ( var(Unit) ->
    ValueQuery=Value;
    ValueQuery=unit(Value,Unit) ).

triple_query_unify2_(Value,Unit,unit(Value1,Unit)) :-
  !, triple_query_unify2_(Value,Unit,Value1).

triple_query_unify2_(Value,_Unit,Value1) :-
  strip_type_(Value,_,Value0),
  ( var(Value1) -> 
    Value1=Value0 ;
    strip_type_(Value1,_,Value0) ).

		 /*******************************
		 *	   .....              	*
		 *******************************/

%%
mng_query_value_(Query,Operator,Value,Unit) :-
  % get operator
  strip_operator_(Query,Operator0,Query0),
  operator_mapping_(Operator0,Operator),
  % get unit if any
  strip_unit_(Query0,Unit,Query1),
  % finally get value
  strip_type_(Query1,Type0,Value0),
  type_mapping_(Type0,MngType),
  Value=..[MngType,Value0].

%%
% For some properties it is useful to store a hierarchy of values 
% instead of a single one.
%
fact_value_hierarchy_(P,string(Value),List) :-
  taxonomical_property(P,RangeType),!,
  ( RangeType=class ->
    findall(string(Sup), tripledb_subclass_of(Value,Sup), List) ;
    RangeType=property ->
    findall(string(Sup), tripledb_subproperty_of(Value,Sup), List) ;
    fail
  ).
fact_value_hierarchy_(_,V0,[V0]).

%%
% Insert an additional scope into a list of scopes
% stored in a triple document.
% The predicate fails in case the new scope is
% already subsumed by an existing scope.
%
update_scope_(Original,New,Updated) :-
  member(Old,Original),
  % no update needed if all facts in New are also
  % contained in Old.
  subscope_of(New,Old),!,
  fail.

update_scope_(Original,New,Updated) :-
  update_scope1_(Original,New,Updated).

update_scope1_([],New,[New]) :- !.
update_scope1_([X|Xs],New,Updated) :-
  scope_merge(X,New,Merged),!,
  update_scope1_(Xs,Merged,Updated).
update_scope1_([X|Xs],New,[X|Updated]) :-
  update_scope1_(Xs,New,Updated).

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
strip_operator_( =(X), =,X).
strip_operator_(>=(X),>=,X).
strip_operator_(=<(X),=<,X).
strip_operator_( <(X), <,X).
strip_operator_( >(X), >,X).
strip_operator_(    X, =,X).

%%
% TODO: better use units as replacement of type.
%          I guess all qudt units are double basetype?
strip_unit_(unit(X,Unit),Unit,X).
strip_unit_(X,_,X).
  
%%
type_mapping_(float,  double).
type_mapping_(double, double).
type_mapping_(long,   int).
type_mapping_(int,    int).
type_mapping_(short,  int).
type_mapping_(byte,   int).
type_mapping_(string, string).

%%
strip_type_(Term,Type,X) :-
  compound(Term),!,
  Term=..[Type,X],
  type_mapping_(Type,_).
strip_type_(X,double,X) :- number(X),!.
% FIXME var(X) always ends in string, but holds takes care of setting type
%                  better do not require type in query!
strip_type_(X,string,X).


		 /*******************************
		 *	   PROPAGATION       *
		 *******************************/
% TODO: cache_invalidate is model dependent and should not be done here,
%           it should only be triggered through tell.

%%
%
%
% FIXME: need to invalidate cache with more properties
propagate_assertion_(rdfs:subClassOf,S,string(O)) :-
  % invalidate all subclass-of inferences when
  % a new subclass-of axiom was added
  tripledb_cache_invalidate(subclass_of),
  % find parents
  findall(string(X), tripledb_subclass_of(O,X), Parents),
  % update type of instances of class S
  tripledb(DB,TriplesColl,_),
  mng_update(DB,TriplesColl,['o*',S],
    ['$addToSet',['o*',['$each',array(Parents)]]]),
  !.
propagate_assertion_(rdfs:subPropertyOf,S,string(O)) :-
  % invalidate all subproperty-of inferences when
  % a new subproperty-of axiom was added
  tripledb_cache_invalidate(subproperty_of),
  % find parents
  findall(string(X), tripledb_subproperty_of(O,X), Parents),
  % update triples asserted for property S
  tripledb(DB,TriplesColl,_),
  mng_update(DB,TriplesColl,['p*',S],
    ['$addToSet',['p*',['$each',array(Parents)]]]),
  !.
propagate_assertion_(_,_,_).

%%
%
%
propagate_deletion_(rdfs:subClassOf,S,string(O)) :-
  tripledb(DB,TriplesColl,_),
  % update o* for all triples where o is a subclass of S
  forall(
    tripledb_subclass_of(X,S),
    ( findall(string(Y), tripledb_subclass_of(X,Y), Parents),
      mng_update(DB,TriplesColl,['o',X],
            ['o*',['$eq',array(Parents)]])
    )
  ),!.
propagate_deletion_(rdfs:subPropertyOf,S,string(O)) :-
  tripledb(DB,TriplesColl,_),
  % update p* for all triples where p is a subproperty of S
  forall(
    tripledb_subproperty_of(X,S),
    ( findall(string(Y), tripledb_subproperty_of(X,Y), Parents),
      mng_update(DB,TriplesColl,['p',X],
            ['p*',['$eq',array(Parents)]])
    )
  ),!.
propagate_deletion_(_,_,_).


		 /*******************************
		 *	   CACHING       *
		 *******************************/

%% 
% @implements 'db/itripledb'
%
%
tripledb_cache_get(Query,Modules) :-
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
tripledb_cache_add(Module,Predicate,Query) :-
  cachedb(DB,Coll),
  mng_store(DB,Coll,_{
        query:     string(Query),
        module:    string(Module),
        predicate: string(Predicate)
  }).

%% 
% @implements 'db/itripledb'
%
tripledb_cache_invalidate(Predicate) :-
  cachedb(DB,Coll),
  mng_cursor_create(DB,Coll,Cursor),
  mng_cursor_filter(Cursor,['predicate',string(Predicate)]),
  mng_cursor_erase(Cursor),
  mng_cursor_destroy(Cursor).
