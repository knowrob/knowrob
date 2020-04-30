:- module(mng_tripledb,
    [ implements('db/itripledb')
    ]).
/** <module> Triple store backend using mongo DB.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db'), 
        [ rdf_meta/1 ]).
:- use_module(library('utility/filesystem'), 
        [ path_concat/2 ]).

:- use_module('./client.pl').

%%
tripledb(DB,'tripledb') :-
  mng_db_name(DB).

%% 
% @implements 'db/itripledb'
%
tripledb_init :-
  tripledb(DB,Coll),
  % TODO: look deeper into creating indices
  % TODO: scopes should define their indices
  mng_index_create(DB,Coll,
        ['s',
         'p*',
         'o*',
         'scope.graph',
         'scope.temporal.since',
         'scope.temporal.until'
        ]).

%% 
% @implements 'db/itripledb'
%
tripledb_import(Dir) :-
  tripledb(DB,_Coll),
  path_concat(Dir,tripledb,Dir0),
  mng_restore(DB,Dir0).

%% 
% @implements 'db/itripledb'
%
tripledb_export(Dir) :-
  tripledb(_DB,Coll),
  path_concat(Dir,tripledb,Dir0),
  mng_export_collection(Coll,Dir0).

%% 
% @implements 'db/itripledb'
%
tripledb_whipe :-
  tripledb(DB,Coll),
  mng_drop(DB,Coll).

%% 
% @implements 'db/itripledb'
%
tripledb_load_rdf(RDF_Data,Scope) :-
  % TODO: bulk insert instead
  % TODO: support triple stream
  forall(
    member(rdf(S,P,O),RDF_Data),
    tripledb_tell(S,P,O,Scope)
  ).

%% 
% @implements 'db/itripledb'
%
tripledb_tell(Subject,Property,Value,Scope) :-
  %%
  findall(string(Y), tripledb_subproperty_of(Property,Y), Properties),
  %%
  findall(Key-Val, (
    (Key='s',  Val=string(Subject));
    (Key='p',  Val=string(Property));
    (Key='p*', Val=array(Properties));
    (get_fact_value_pair_(Property,Value,Key,Val));
    (get_scope_pair_(Scope,Key,['=',Val]))
  ), Pairs),
  dict_pairs(Dict,Pairs),
  %%
  tripledb(DB,Coll),
  mng_store(DB,Coll,Dict),
  %%
  tripledb_tell2(Property,Subject,Value).

:- rdf_meta tripledb_tell2(r,+,+).

tripledb_tell2(rdfs:subClassOf,S,O) :-
  % find parents
  findall(string(X), tripledb_subclass_of(O,X), Parents),
  % update type of instances of class S
  tripledb(DB,Coll),
  mng_update(DB,Coll,['o*',S],
    ['$addToSet',['o*',['$each',array(Parents)]]]),
  !.

tripledb_tell2(rdfs:subPropertyOf,S,O) :-
  % find parents
  findall(string(X), tripledb_subproperty_of(O,X), Parents),
  % update triples asserted for property S
  tripledb(DB,Coll),
  mng_update(DB,Coll,['p*',S],
    ['$addToSet',['p*',['$each',array(Parents)]]]),
  !.

tripledb_tell2(_,_,_).

%% 
% @implements 'db/itripledb'
%
tripledb_ask(Subject,Property,Value,QScope,FScope) :-
  setup_call_cleanup(
    % setup: create a query cursor
    triple_query_cursor_(Subject,Property,Value,QScope,Cursor),
    % call: find matching document
    triple_query_unify_(Cursor,Subject,Property,Value,FScope),
    % cleanup: destroy cursor again
    mng_cursor_destroy(Cursor)
  ).

%% 
% @implements 'db/itripledb'
%
tripledb_forget(Subject,Property,Value,Scope) :-
  triple_query_cursor_(Subject,Property,Value,Scope,Cursor),
  mng_cursor_erase(Cursor),
  mng_cursor_destroy(Cursor),
  % RDFS update
  tripledb_forget2(Property,Subject,Value).

:- rdf_meta tripledb_forget2(r,+,+).

tripledb_forget2(rdfs:subClassOf,S,O) :-
  tripledb(DB,Coll),
  % update o* for all triples where o is a subclass of S
  forall(
    tripledb_subclass_of(X,S),
    ( findall(string(Y), tripledb_subclass_of(X,Y), Parents),
      mng_update(DB,Coll,['o',X],
            ['o*',['$eq',array(Parents)]])
    )
  ),!.

tripledb_forget2(rdfs:subPropertyOf,S,O) :-
  tripledb(DB,Coll),
  % update p* for all triples where p is a subproperty of S
  forall(
    tripledb_subproperty_of(X,S),
    ( findall(string(Y), tripledb_subproperty_of(X,Y), Parents),
      mng_update(DB,Coll,['p',X],
            ['p*',['$eq',array(Parents)]])
    )
  ),!.

tripledb_forget2(_,_,_).

%% 
% @implements 'db/itripledb'
%
tripledb_type_of(Entity,Type) :-
  tripledb_ask(Entity,rdf:type,Type,_{}).

%% 
% @implements 'db/itripledb'
%
tripledb_subclass_of(Class,Class).
tripledb_subclass_of(Subclass,Class) :-
  tripledb_ask(Subclass,rdfs:subClassOf,Class,_{}).

%% 
% @implements 'db/itripledb'
%
tripledb_subproperty_of(Property,Property).
tripledb_subproperty_of(Subproperty,Property) :-
  tripledb_ask(Subproperty,rdfs:subPropertyOf,Property,_{}).

		 /*******************************
		 *	   .....              	*
		 *******************************/

%% create a query cursor
triple_query_cursor_(Subject,Property,Value,Scope,Cursor) :-
  tripledb(DB,Collection),
  mng_cursor_create(DB,Collection,Cursor),
  % TODO: allow to configue query
  % - mng_cursor_descending(Cursor,t0),
  % - mng_cursor_limit(Cursor,1).
  ( \+ground(Subject)  ; mng_cursor_filter(Cursor,['s',Subject]) ),
  ( \+ground(Property) ; mng_cursor_filter(Cursor,['p*',Property]) ),
  ( \+ground(Value)    ; mng_cursor_filter_value_(Cursor,'o*',Value) ),
  % filter scope
  forall(
    get_scope_pair_(Scope,Scope_Key,Scope_Value),
    mng_cursor_filter(Cursor,[Scope_Key,['$eq',Scope_Value]])
  ).

%%
triple_query_unify_(Cursor,Subject,Property,Value,FScope) :-
  mng_cursor_materialize(Cursor,Doc),
  ( ground(Subject)  -> true ; mng_get_dict('s',Doc,Subject) ),
  ( ground(Property) -> true ; mng_get_dict('p',Doc,Property) ),
  ( ground(Value)    -> true ; mng_get_dict('o',Doc,Value) ),
  % get the fact scope
  ( mng_get_dict('scope',Doc,FScope) -> true; FScope=_{} ).

		 /*******************************
		 *	   .....              	*
		 *******************************/

%%
mng_cursor_filter_value_(Cursor,Key,Query) :-
  mng_query_value_(Query,MngOperator,MngValue),
  mng_cursor_filter(Cursor,[Key,[MngOperator,MngValue]])!.

%%
mng_query_value_(Query,Operator_mng,Value_mng) :-
  % FIXME: this demands all values as $op($val,%type), maybe relax?
  Query=..[Operator,Value,Type],
  operator_mapping_(Operator,Operator_mng),
  type_mapping_(Type,MngType),
  Value_mng=..[MngType,Value].

%%
get_fact_value_pair_(Property,=(Value,Type),K,V) :-
  type_mapping_(Type,MngType),
  MngValue=..[MngType,Value],
  % FIXME: asserting all super-classes is NOT OK for 
  %           e.g. disjointness (disjointness holds for all sub-classes instead!)
  % - maybe model shoud define how properties should be handled
  fact_value_hierarchy_(Property,MngValue,MngHierarchy),
  ( (K='o',  V=MngValue);
    (K='o*', V=array(MngHierarchy))
  ).

%%
%
fact_value_hierarchy_(string(Value),List) :-
  tripledb_type_of(Value,rdfs:'Class'),!,
  findall(string(X), tripledb_subclass_of(Value,X), List).

fact_value_hierarchy_(string(Value),List) :-
  tripledb_type_of(Value,rdf:'Property'),!,
  findall(string(X), tripledb_subproperty_of(Value,X), List).

fact_value_hierarchy_(V0,[V0]).

%%
get_scope_pair_(Scopes,Path,Value) :-
  get_dict(ScopeName,Scopes,ScopeData),
  get_scope_pair2_(ScopeData,SubPath,Value),
  atomic_list_concat([scope,ScopeName,SubPath],'.',Path).

get_scope_pair2_(Scope,Path,Value) :-
  is_dict(Scope),!,
  get_dict(Key,Scope,Data),
  get_scope_pair2_(Data,SubPath,Value),
  ( SubPath='' -> Path=Key;
    atomic_list_concat([Key,SubPath],'.',Path)
  ).

get_scope_pair2_(Query,'',[Operator,Value]) :-
  mng_query_value_(Query,Operator,Value).

%%
operator_mapping_('=', '$eq').
operator_mapping_('>=','$gte').
operator_mapping_('=<','$lte').

%%
type_mapping_(float,  double).
type_mapping_(double, double).
type_mapping_(long,   int).
type_mapping_(int,    int).
type_mapping_(short,  int).
type_mapping_(byte,   int).
type_mapping_(string, string).
