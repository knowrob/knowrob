:- module(mng_triples,
    [ triple_init/0,
      triple_drop/0,
      triple_erase(r,r,t,+,+),
      mng_triple_doc(t,-,t)
    ]).
/** <module> Triple store backend using mongo DB.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db'),
	[ rdf_meta/1 ]).
:- use_module(library('db/subgraph')).
:- use_module(library('db/scope')).

:- use_module('../client.pl').

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

%%
filter_graph_('*', _)    :- !, fail.
filter_graph_('user', _) :- !, fail.
filter_graph_(=(GraphName),
	['graph',string(GraphName)]) :- !.
filter_graph_(GraphName,
	['graph',['$in',array(Graphs)]]) :-
	tripledb_get_supgraphs(GraphName,Graphs).

%%
filter_scope_(Scope,[Key,Filter]) :-
	get_scope_query_(Scope,Key,Filter).

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

