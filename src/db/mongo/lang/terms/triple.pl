:- module(mng_term_triple, []).

:- use_module(library('semweb/rdf_db'),
	    [ rdf_meta/1
	    ]).
:- use_module(library('db/mongo/lang/compiler')).
:- use_module(library('db/mongo/lang/query')).
:- use_module(library('db/mongo/tripledb/triples'),
		[ mng_triple_doc/3
		]).
:- use_module(library('lang/scopes/temporal'),
		[ time_scope_data/2
		]).

:- rdf_meta(triple_tell(t,t,t,t,t,-)).

%% register query commands
:- mng_query_command(triple).

%%
% expose subject/predicate/object argument variables.
%
mng_compiler:step_var(
		triple(S,P,O),
		[Key, Var]) :-
	(	triple_var_(S, [Key, Var])
	;	triple_var_(P, [Key, Var])
	;	triple_var_(O, [Key, Var])
	).

%%
triple_var_(Arg, [Key, Var]) :-
	once((
		( nonvar(Arg), Arg=(_->Var) )
	;	mng_strip_type(Arg,_,Var)
	)),
	mng_compiler:var_key(Var, Key).

%%
% ask(triple(S,P,O)) uses $lookup to join input documents with
% the ones matching the triple pattern provided.
%
mng_compiler:step_compile(
		triple(S,P,O),
		Context,
		Pipeline) :-
	option(ask, Context),
	% get the collection name
	% TODO: do this during expansion
	(	option(collection(Coll), Context)
	;	mng_get_db(_DB, Coll, 'triples')
	),
	% extend the context
	Context0 = [
		property(P),
		collection(Coll)
	|	Context
	],
	% compute steps of the aggregate pipeline
	findall(Step,
		% filter out documents that do not match the triple pattern.
		% this is done using $match or $lookup operators.
		(	lookup_(triple(S,P,O), Context0, Step)
		% conditionally needed to harmonize 'next' field
		;	set_next_(Context0, Step)
		% add additional results if P is a transitive property
		;	transitivity_(Context0, Step)
		% add additional results if P is a reflexive property
		;	reflexivity_(Context0, Step)
		% at this point 'next' field holds an array of matching documents
		% that is unwinded here.
		;	Step=['$unwind',string('$next')]
		% compute the intersection of scope so far with scope of next document
		;	scope_step(Context0, Step)
		% project new variable groundings
		;	project_(triple(S,P,O), Context0, Step)
		),
		Pipeline
	).

%%
% tell(triple(S,P,O)) uses $lookup to find matching triples
% with overlapping scope which are toggled to be removed in next stage.
% then the union of their scopes is computed and used for output document.
%
mng_compiler:step_compile(
		triple(S,rdf:type,Cls),
		Context, Pipeline) :-
	option(tell, Context),
	!,
	% handle the case var(S), in that case, generate a new symbol
	% TODO: calls is_resource. either table it or gen name in mongo
	once((nonvar(S) ; lang_is_a:unique_name(Cls, S))),
	%
	triple_tell(
		triple(S,rdf:type,Cls), _,
		parents(Cls,rdfs:subClassOf),
		[ array([string(rdf:type)]),
		  string('$parents') ],
		Context, Pipeline).

mng_compiler:step_compile(
		triple(Sub,rdfs:subClassOf,Sup),
		Context, Pipeline) :-
	option(tell, Context),
	!,
	triple_tell(
		triple(Sub,rdfs:subClassOf,Sup), _,
		parents(Sup,rdfs:subClassOf),
		[ array([string(rdfs:subClassOf)]),
		  string('$parents') ],
		[ propagate|Context ],
		Pipeline).

mng_compiler:step_compile(
		triple(Sub,rdfs:subPropertyOf,Sup),
		Context, Pipeline) :-
	option(tell, Context),
	!,
	triple_tell(
		triple(Sub,rdfs:subPropertyOf,Sup), _,
		parents(Sup,rdfs:subPropertyOf),
		[ array([string(rdfs:subPropertyOf)]),
		  string('$parents') ],
		[ propagate|Context ],
		Pipeline).

mng_compiler:step_compile(
		triple(S,P,O),
		Context, Pipeline) :-
	option(tell, Context),
	!,
	triple_tell(
		triple(S,P,O), MngValue,
		parents(P,rdfs:subPropertyOf),
		[ string('$parents'),
		  array([MngValue]) ],
		Context, Pipeline).

%%
triple_tell(
		triple(S,P,O),
		MngValue,
		parents(Child,Property),
		[Pstar, Ostar],
		Context, Pipeline) :-
	option(graph(Graph), Context, user),
	% get the collection name
	mng_get_db(_DB, Coll, 'triples'),
	% strip the value, assert that operator must be $eq
	% all others do not make sense fro tell.
	% FIXME: move to client.pl
	mng_triples:mng_query_value_(O,'$eq',MngValue,Unit),
	% extend the context
	Context0 = [
		property(P),
		collection(Coll)
	|	Context],
	% build triple docuemnt
	TripleDoc0=[
		['s', string(S)],
		['p', string(P)],
		['o', MngValue],
		['p*', Pstar],
		['o*', Ostar],
		%['o*', array([MngValue])],
		['graph', string(Graph)],
		['scope', string('$v_scope')]
	],
	(	ground(Unit)
	->	TripleDoc=[['unit',string(Unit)]|TripleDoc0]
	;	TripleDoc=TripleDoc0
	),
	% compute steps of the aggregate pipeline
	% TODO: if just one document, update instead of delete
	findall(Step,
		% TODO: assign v_scope field. 
		(	set_scope_(Context0, Step)
		% lookup documents that overlap with triple into 'next' field,
		% and toggle their delete flag to true
		;	lookup_overlapping_(TripleDoc0, Context0, Step)
		% lookup parent documents into the 'parents' field
		;	lookup_parents_(Child, Property, Context0, Step)
		% compute the union of scopes in next
		;	scope_union_(Step)
		% add triples to triples array that have been queued to be removed
		;	array_concat_('triples', string('$next'), Step)
		% add merged triple document to triples array
		;	array_concat_('triples', array([TripleDoc]), Step)
		;	(	option(propagate,Context),
				propagate_tell_(S, Context0, Step)
			)
		),
		Pipeline
	).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% FILTERING documents based on triple pattern
%%%%%%%%%%%%%%%%%%%%%%%

%%
lookup_overlapping_(TripleDoc, Context, ['$lookup', [
			['from',string(Coll)],
			% create a field "next" with all matching documents
			['as',string('next')],
			% get matching documents
			['pipeline',array(Pipeline)]
		]]) :-
	option(collection(Coll), Context),
	% read triple data
	memberchk(['s',S], TripleDoc),
	memberchk(['p',P], TripleDoc),
	memberchk(['o',O], TripleDoc),
	% read scope data
	option(scope(Scope), Context),
	time_scope_data(Scope,[Since,Until]),
	% build pipeline
	findall(Step,
		% $match s,p,o and overlapping scope
		(	Step=['$match',[
				['s',S], ['p',P], ['o',O],
				['scope.time.since',['$lte',double(Until)]],
				['scope.time.until',['$gte',double(Since)]]
			]]
		% only keep scope field
		;	Step=['$project',[['scope',int(1)]]]
		% toggle delete flag
		;	Step=['$set',['delete',bool(true)]]
		),
		Pipeline
	).

%%
lookup_(Triple, Context, Step) :-
	mng_triple_doc(Triple, QueryDoc, Context),
	% lookup matching documents and store in 'next' field
    (	lookup_1(Triple, QueryDoc, Context, Step)
    % unwind the 'next' field
    ;	lookup_unwind_(Context, Step)
    ).

%% unwind $next array field.
lookup_unwind_(Context,
	['$unwind',[
		['path', string('$next')],
		['preserveNullAndEmptyArrays',bool(true)]
	]]) :-
	memberchk(ignore,Context),
	!.
lookup_unwind_(_,
	['$unwind',string('$next')]).

%% 
lookup_1(triple(S,P,O), QueryDoc, Context, Lookup) :-
	% read options
	memberchk(outer_vars(QueryVars), Context),
	memberchk(collection(Coll), Context),
	%
	findall([Key,Field],
		(	member([Field,Arg], [[s,S],[p,P],[o,O]]),
			triple_var_(Arg, [Key, _Var])
		),
		TripleVars
	),
	% find all joins with input documents
	findall([Field_j,Value_j],
		(	member([Field_j,Value_j], TripleVars),
			member([Field_j,_], QueryVars)
		),
		Joins),
	% pass input document value to lookup
	findall([Let_key,string(Let_val)],
		(	member([Let_key,_],Joins),
			atom_concat('$',Let_key,Let_val)
		),
		LetDoc),
	% perform the join operation (equals the input document value)
	findall(['$eq', array([string(Match_key),string(Match_val)])],
		% { $eq: [ "$s",  "$$R" ] },
		(	member([Join_var,Join_field],Joins),
			atom_concat('$',Join_field,Match_key),
			atom_concat('$$',Join_var,Match_val)
		),
		MatchDoc),
	%
	(	MatchDoc=[]
	->	Match=['$match', QueryDoc]
	;	Match=['$match', [['$expr', ['$and', array(MatchDoc)]] | QueryDoc ]]
	),
	(	member(limit(Limit),Context)
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
set_next_(Context, Step) :-
	%% this step is used to harmonize documents
	\+ memberchk(transitive,Context),
	(	% assign *start* field in case of reflexive property
		(	memberchk(reflexive,Context),
			Step=['$set', ['start', string('$next')]]
		)
	;	% transform next into single-element array
		Step=['$set', ['next', array([string('$next')])]]
	).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% PROPERTY SEMANTICS
%%%%%%%%%%%%%%%%%%%%%%%

%%
transitivity_(Context, Step) :-
	% read options
	memberchk(transitive, Context),
	memberchk(collection(Coll), Context),
	memberchk(property(Property), Context),
	% yield steps
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

%%
% FIXME: this creates redundant results for the case of graph queries
%        that receive multiple documents with the same subject as input.
reflexivity_(Context, Step) :-
	memberchk(reflexive,Context),
	Step=['$set', ['next', ['$concatArrays',
		array([string('$next'), array([[
			['s',string('$start.s')],
			['p',string('$start.p')],
			['o',string('$start.s')],
			['scope',string('$start.scope')]
		]])])
	]]].

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% PROJECTION
%%%%%%%%%%%%%%%%%%%%%%%

project_(triple(S,P,O), Context, ['$project', ProjectDoc]) :-
	% read options
	memberchk(outer_vars(QueryVars), Context),
	%
	findall([Key,Field],
		(	member([Field,Arg], [[s,S],[p,P],[o,O]]),
			triple_var_(Arg, [Key, _Var])
		),
		TripleVars
	),
	% 
	findall([Pr_Key,Pr_Value],(
		% copy scope
		(	Pr_Key='v_scope',
			Pr_Value=string('$v_scope')
		)
		% copy value of var e.g. { 'S': '$S' }
	;	(	member([Pr_Key,_], QueryVars),
			atom_concat('$',Pr_Key,Pr_Value0),
			Pr_Value=string(Pr_Value0)
		)
		% set new value of var e.g. { 'S': '$next.s' }
	;	(	member([Pr_Key, Field],TripleVars),
			\+ member([Pr_Key,_], QueryVars),
			project_1(Context, Field, Pr_Value)
		)
	), ProjectDoc).

%% project a grounded field.
project_1(Context, Field,
		['$cond',array([
			['$not', array([string(Pr_Value)]) ],
			% HACK: REMOVE creates problems in next project step,
			% so rather choose some dump value.
			string('null'),
			%string('$$REMOVE'),
			string(Pr_Value)
		])]) :-
	memberchk(ignore,Context),
	!,
	atom_concat('$next.',Field,Pr_Value).
project_1(_,Field,string(Pr_Value)) :-
	!,
	atom_concat('$next.',Field,Pr_Value).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% SCOPE
%%%%%%%%%%%%%%%%%%%%%%%

%%
set_scope_(Context, ['$set',['v_scope',[
		['time',[
			['since', double(Since)],
			['until', double(Until)]
		]]]]]) :-
	% read scope data
	option(scope(Scope), Context),
	time_scope_data(Scope,[Since,Until]).

%%
scope_union_(Step) :-
	% create array with all scope.time.since values
	(	Step=['$set',['num_array',['$map',[
			['input', string('$next')],
			['in', string('$$this.scope.time.since')]
		]]]]
	;	array_concat_('num_array',
			array([string('$v_scope.time.since')]),
			Step)
	% set the minimum of since values
	;	Step=['$set',['v_scope.time.since',
			['$min',string('$num_array')]
		]]
	% create array with all scope.time.until values
	;	Step=['$set',['num_array',['$map',[
			['input', string('$next')],
			['in', string('$$this.scope.time.until')]
		]]]]
	;	array_concat_('num_array',
			array([string('$v_scope.time.until')]),
			Step)
	% set the maximum of until values
	;	Step=['$set',['v_scope.time.until',
			['$max',string('$num_array')]
		]]
	;	Step=['$unset',string('num_array')]
	).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% PROPAGATION
%%%%%%%%%%%%%%%%%%%%%%%

%%
lookup_parents_(Child, Property, Context, Step) :-
	memberchk(collection(Coll), Context),
	% first, lookup matching documents and yield o* in parents array
	(	Step=['$lookup', [
			['from',string(Coll)],
			% create a field "next" with all matching documents
			['as',string('parents')],
			% get matching documents
			['pipeline',array([
				['$match', [
					['s',string(Child)],
					['p',string(Property)]
				]],
				['$project', [['o*', int(1)]]],
				['$unwind', string('$o*')]
			])]
		]]
	% convert parents from list of documents to list of strings.
	;	Step=['$set',['parents',['$map',[
			['input',string('$parents')],
			['in',string('$$this.o*')]
		]]]]
	% also add child to parents list
	;	array_concat_('parents', array([string(Child)]), Step)
	).

%%
propagate_tell_(S, Context, Step) :-
	memberchk(collection(Coll), Context),
	% the inner lookup matches documents with S in o*
	findall(X,
		% match every document with S in o*
		(	X=['$match', [['o*',string(S)]]]
		% and add parent field from input documents to o*
		;	array_concat_('o*', string('$$parents'), X)
		% only replace o*
		;	X=['$project',[['o*',int(1)]]]
		),
		Inner),
	% first, lookup matching documents and update o*
	(	Step=['$lookup', [
			['from',string(Coll)],
			% create a field "next" with all matching documents
			['as',string('next')],
			% make fields from input document accessible in pipeline
			['let',['parents','$parents']],
			% get matching documents
			['pipeline',array(Inner)]
		]]
	% second, add each document to triples array
	;	array_concat_('triples', string('$next'), Step)
	).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% helper
%%%%%%%%%%%%%%%%%%%%%%%

%%
array_concat_(Key,Arr,['$set',
		[Key,['$setUnion',
			array([string(Arr0),Arr])]
		]]) :-
	atom_concat('$',Key,Arr0).

