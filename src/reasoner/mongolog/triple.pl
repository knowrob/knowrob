:- module(mongolog_triple,
        [ mng_triple_doc(t,-,t),
          triple(t,t,t),
          get_unique_name(r,-),
          is_unique_name(r)
        ]).
/** <module> Handling of triples in query expressions.

The following predicates are supported:

| Predicate            | Arguments |
| ---                  | ---       |
| triple/3         | ?Subject, ?Property, ?Value |

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db'),
		[ rdf_meta/1, rdf_equal/2 ]).
:- use_module(library('reasoner'),
		[ current_reasoner_module/1 ]).
:- use_module('client').
:- use_module(library('mongolog/mongolog')).
:- use_module(library('mongolog/mongolog_test')).
:- use_module(library('semweb')).

:- rdf_meta(taxonomical_property(r)).
:- rdf_meta(triple(t,t,t)).

%% register query commands
:- mongolog:add_command(triple).


%%
mongolog:step_expand(project(triple(S,P,O)), assert(triple(S,P,O))) :- !.

%%
mongolog:step_compile(assert(triple(S,P,term(O))), Ctx, Pipeline, StepVars) :-
	% HACK: convert term(A) argument to string.
	%       it would be better to store lists/terms directly without conversion.
	ground(O),!,
	( atom(O) -> Atom=O ; term_to_atom(O, Atom) ),
	mongolog:step_compile(assert(triple(S,P,string(Atom))), Ctx, Pipeline, StepVars).

%%
mongolog:step_compile(assert(triple(S,P,O)), CtxIn, Pipeline, StepVars) :-
	% add step variables to compile context
	triple_step_vars(triple(S,P,O), CtxIn, StepVars0),
	mongolog:add_assertion_var(StepVars0, StepVars),
	merge_options([step_vars(StepVars)], CtxIn, Ctx),
	% add additional options to the compile context
	extend_context(triple(S,P,O), P1, Ctx, Ctx0),
	option(collection(Collection), Ctx0),
	option(query_scope(Scope), Ctx0),
	triple_graph_for_assert(Ctx0, Graph),
	% throw instantiation_error if one of the arguments was not referred to before
	mongolog:all_ground([S,O], Ctx),
	% resolve arguments
	mongolog:var_key_or_val(S, Ctx, S_query),
	mongolog:var_key_or_val(P1, Ctx, P_query),
	mongolog:var_key_or_val(O, Ctx, V_query),
	findall(Field, (
		Field=['s', S_query] ;
		Field=['p', P_query] ;
		Field=['o', V_query] ;
		Field=['graph', string(Graph)] ;
		( option(since(Since), Scope), option(until(Until), Scope),
		  mng_typed_value(Since, SinceTyped),
		  mng_typed_value(Until, UntilTyped),
		  Field=['scope', [['time',[['since', SinceTyped], ['until', UntilTyped]]]]]
		) ;
		( option(since(Since), Scope), \+ option(until(Until), Scope),
		  mng_typed_value(Since, SinceTyped),
		  Field=['scope', [['time',[['since', SinceTyped]]]]]
		) ;
		( \+ option(since(Since), Scope), option(until(Until), Scope),
		  mng_typed_value(Until, UntilTyped),
		  Field=['scope', [['time',[['until', UntilTyped]]]]]
		)
	), Fields),
	% create assertion pipeline
	findall(Step,
		mongolog:add_assertion(Fields, Collection, Step),
		Pipeline).


mongolog:step_compile(triple(S,P,term(O)), Ctx, Pipeline, StepVars) :-
	% HACK: convert term(A) argument to string.
	%       it would be better to store lists/terms directly without conversion.
	ground(O),!,
	( atom(O) -> Atom=O ; term_to_atom(O, Atom) ),
	mongolog:step_compile(triple(S,P,string(Atom)), Ctx, Pipeline, StepVars).

%%
mongolog:step_compile(triple(S,P,O), Ctx, Pipeline, StepVars) :-
	% add step variables to compile context
	triple_step_vars(triple(S,P,O), Ctx, StepVars),
	merge_options([step_vars(StepVars)], Ctx, Ctx0),
	% create pipeline
	compile_ask(triple(S,P,O), Ctx0, Pipeline).

%%
triple_step_vars(triple(S,P,O), Ctx, StepVars) :-
	(	bagof(Var,
			(	mongolog:goal_var([S,P,O], Ctx, Var)
			;	mongolog:context_var(Ctx, Var)
			% HACK: remember that variable is wrapped in term/1
			;	(	nonvar(O),
					O=term(O1),
					var(O1),
					mongolog:var_key(O1, Ctx, Key),
					Var=[Key,term(O1)]
				)
			),
			StepVars)
	;	StepVars=[]
	),!.

%%
% ask(triple(S,P,O)) uses $lookup to join input documents with
% the ones matching the triple pattern provided.
%
compile_ask(triple(S,P,O), Ctx, Pipeline) :-
	% add additional options to the compile context
	extend_context(triple(S,P,O), P1, Ctx, Ctx0),
	findall(LookupStep,
		lookup_triple(triple(S,P1,O), Ctx0, LookupStep),
		LookupSteps),
	LookupSteps \== [],
	% compute steps of the aggregate pipeline
	findall(Step,
		% filter out documents that do not match the triple pattern.
		% this is done using $match or $lookup operators.
		(	member(Step, LookupSteps)
		% compute the intersection of scope so far with scope of next document
		;	mongolog_scope_intersect('v_scope',
				string('$next.scope.time.since'),
				string('$next.scope.time.until'),
				Ctx0, Step)
		% update "uncertain" flag
		;	Step=['$set', [['v_scope.uncertain', [['$or', array([
				string('$v_scope.uncertain'),
				string('$next.uncertain')
			])]]]]]
		% the triple doc contains parents of the P at p*
		% and parents of O at o*.
		% these can be unwinded to yield a solution for each parent.
		;	unwind_parents(Ctx0, Step)
		% project new variable groundings
		;	set_triple_vars(S,P1,O,Ctx0,Step)
		% remove next field again
		;	Step=['$unset',string('next')]
		),
		Pipeline
	).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% LOOKUP triple documents
%%%%%%%%%%%%%%%%%%%%%%%

%%
lookup_triple(triple(S,P,V), Ctx, Step) :-
	\+ memberchk(transitive, Ctx),
	memberchk(collection(Coll), Ctx),
	memberchk(step_vars(StepVars), Ctx),
	mng_triple_doc(triple(S,P,V), QueryDoc, Ctx),
	(	memberchk(['s',_],QueryDoc)
	->	StartValue='$start.s'
	;	StartValue='$start.o'
	),
	%
	(	taxonomical_property(P)
	->	( Key_p='$p',  Key_o='$o*' )
	;	( Key_p='$p*', Key_o='$o' )
	),
	findall(MatchQuery,
		% first match what is grounded in compile context
		(	MatchQuery=QueryDoc
		% next match variables grounded in call context
		;	(	member([Arg,FieldValue],[[S,'$s'],[P,Key_p],[V,Key_o]]),
				triple_arg_var(Arg, ArgVar),
				mongolog:var_key(ArgVar, Ctx, ArgKey),
				atom_concat('$$',ArgKey,ArgValue),
				atom_concat(ArgValue,'.type',ArgType),
				triple_arg_value(Arg, ArgValue, FieldValue, Ctx, ArgExpr),
				MatchQuery=['$expr', ['$or', array([
					% pass through if var is not grounded
					['$eq', array([string(ArgType), string('var')])],
					ArgExpr % else perform a match
				])]]
			)
		;	mongolog_scope_match(Ctx, MatchQuery)
		),
		MatchQueries
	),
	%
	findall(InnerStep,
		% match input triples with query pattern
		(	(	MatchQueries=[FirstMatch]
			->	InnerStep=['$match', FirstMatch]
			;	InnerStep=['$match', ['$and', array(MatchQueries)]]
			)
		% limit results if requested
		;	(	member(limit(Limit),Ctx),
				InnerStep=['$limit',int(Limit)]
			)
		),
		InnerPipeline
	),
	% pass input document values to lookup
	mongolog:lookup_let_doc(StepVars, LetDoc),
	% lookup matching documents and store in 'next' field
    (	Step=['$lookup', [
			['from',string(Coll)],
			['as',string('next')],
			['let',LetDoc],
			['pipeline', array(InnerPipeline)]
		]]
	% add additional results if P is a reflexive property
	;	(	memberchk(reflexive,Ctx),
			(	Step=['$unwind',string('$next')]
			;	Step=['$set', ['start', string('$next')]]
			;	Step=['$set', ['next', array([string('$next')])]]
			;	reflexivity(StartValue, Ctx, Step)
			)
		)
	% at this point 'next' field holds an array of matching documents
	% that is unwinded here.
	;	Step=['$unwind',string('$next')]
	).

%%
lookup_triple(triple(S,P,V), Ctx, Step) :-
	% read options
	option(transitive, Ctx),
	option(collection(Coll), Ctx),
	mongolog_one_db(_DB, OneColl),
	% infer lookup parameters
	mng_query_value(P,Query_p),
	% TODO: can query operators be supported?
	mng_strip_variable(S, S0),
	mng_strip_variable(V, V0),
	mongolog:var_key_or_val(S0, Ctx, S_val),
	mongolog:var_key_or_val(V0, Ctx, V_val),
	
	% FIXME: a runtime condition is needed to cover the case where S was referred to in ignore'd goal that failed.
	(	has_value(S0,Ctx)
	->	( Start=S_val, To='s', From='o', StartValue='$start.s' )
	;	( Start=V_val, To='o', From='s', StartValue='$start.o' )
	),
	
	% match doc for restring the search
	findall(Restriction,
		(	Restriction=['p*',Query_p]
		% TODO: see how scope can be included in transitive lookups
		%;	graph_doc(Graph,Restriction)
		%;	scope_doc(Scope,Restriction)
		),
		MatchDoc
	),
	% recursive lookup
	(	Step=['$graphLookup', [
			['from',                    string(Coll)],
			['startWith',               Start],
			['connectToField',          string(To)],
			['connectFromField',        string(From)],
			['as',                      string('t_paths')],
			['depthField',              string('depth')],
			['restrictSearchWithMatch', MatchDoc]
		]]
	% $graphLookup does not ensure order, so we need to order by recursion depth
	% in a separate step
	;	Step=['$lookup', [
			['from',     string(OneColl)],
			['as',       string('t_sorted')],
			['let',      [['t_paths', string('$t_paths')]]],
			['pipeline', array([
				['$set',         ['t_paths', string('$$t_paths')]],
				['$unwind',      string('$t_paths')],
				['$replaceRoot', ['newRoot', string('$t_paths')]],
				['$sort',        ['depth', integer(1)]]
			])]
		]]
	;	Step=['$set', ['next', string('$t_sorted')]]
	;	Step=['$set', ['start', ['$arrayElemAt',
			array([string('$next'), integer(0)])
		]]]
	;	Step=['$unset', string('t_paths')]
	;	Step=['$unset', string('t_sorted')]
	% add additional triple in next if P is a reflexive property
	;	reflexivity(StartValue, Ctx, Step)
	% iterate over results
	;	Step=['$unwind',string('$next')]
	% match values with expression given in query
	;	(	To=='s', has_value(V0,Ctx),
			Step=['$match', ['next.o', V_val]]
		)
	).

%%
%
has_value(X, _Ctx) :-
	ground(X),!.
has_value(X, Ctx) :-
	term_variables(X,Vars),
	member(Var,Vars),
	mongolog:is_referenced(Var, Ctx).

%%
reflexivity(StartValue, Ctx, Step) :-
	memberchk(reflexive,Ctx),
	(	Step=['$set', ['t_refl', [
			['s',string(StartValue)],
			['p',string('$start.p')],
			['p*',string('$start.p*')],
			['o',string(StartValue)],
			['o*',array([string(StartValue)])],
			['graph',string('$start.graph')],
			['scope',string('$start.scope')]
		]]]
	;	Step=['$set', ['next', ['$concatArrays',
			array([array([string('$t_refl')]), string('$next')])
		]]]
	;	Step=['$unset', array([string('t_refl'),string('start')])]
	).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% RDFS
%%%%%%%%%%%%%%%%%%%%%%%

%%
taxonomical_property(P) :- var(P),!,fail.
taxonomical_property(Term) :-
	compound(Term),!,
	Term =.. [_Functor,Arg],
	taxonomical_property(Arg).
taxonomical_property(rdf:type).
taxonomical_property(rdfs:subClassOf).
taxonomical_property(rdfs:subPropertyOf).

%%
unwind_parents(Context, Step) :-
	option(pstar, Context),
	(	Step=['$unwind', string('$next.p*')]
	;	Step=['$set', ['next.p', ['$next.p*']]]
	).
unwind_parents(Context, Step) :-
	option(ostar, Context),
	(	Step=['$unwind', string('$next.o*')]
	;	Step=['$set', ['next.o', string('$next.o*')]]
	).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% triple/3 query pattern
%%%%%%%%%%%%%%%%%%%%%%%

%% triple(?Subject, ?Property, ?Value) is nondet.
%
% Query values of a property (and their sub-properties) on some subject in the triple DB.
% If the property is rdfs:subPropertyOf or rdf:type the query returns the values for the 
% subject and their super-class
% 
% The property can be wrapped in one of several options:
% 
%     - transitive(Property) 
%       indicates that the property is transitive
%     - reflexive(Property)
%       indicates that the property is reflexive
%     - pstar(Property)
%       binds the property to one of the values in the p* field in the mongodb
%
% The value can be wrapped in one of several options:
%
%     - ostar(Value)
%       binds the value to one of the values in the o* field in the mongodb
%
% @param Subject The subject of a triple.
% @param Property The predicate of a triple.
% @param Value The object of a triple.
%
triple(S,P,O) :-
	mongolog_call(triple(S,P,O)).

%% mng_triple_doc(+Triple, -Doc, +Context) is semidet.
%
% Translate a triple term into a mongo query document.
%
mng_triple_doc(triple(S,P,V), Doc, Context) :-
	%% read options
	triple_graph_for_query(Context, Graph),
	option(query_scope(Scope), Context, dict{}),
	% special handling for some properties
	(	taxonomical_property(P)
	->	( Key_p='p',  Key_o='o*' )
	;	( Key_p='p*', Key_o='o' )
	),
	% strip term ->(Term,Var)
	mng_strip_variable(S, S1),
	mng_strip_variable(P, P1),
	mng_strip_variable(V, V1),
	% get the query pattern
	% FIXME: mng_query_value may silently fail on invalid input and this rule still succeeds
	findall(X,
		(	( mng_query_value(S1,Query_s), X=['s',Query_s] )
		;	( mng_query_value(P1,Query_p), X=[Key_p,Query_p] )
		;	( mng_query_value(V1,Query_v), \+ is_term_query(Query_v), X=[Key_o,Query_v] )
		;	graph_doc(Graph,X)
		;	mongolog_scope_doc(Scope,X)
		),
		Doc
	),
	% ensure doc has value if input is grounded
	once((\+ ground(S1) ; memberchk(['s',_],Doc))),
	once((\+ ground(P1) ; memberchk([Key_p,_],Doc))),
	once((\+ ground(V1) ; memberchk([Key_o,_],Doc))).

%%
is_term_query([[type,string(compound)],_]).
is_term_query([_, [[type,string(compound)],_]]).

%%
triple_arg_var(Arg, ArgVar) :-
	mng_strip_variable(Arg, X),
	term_variables(X, [ArgVar]).

%%
triple_arg_value(_Arg, ArgValue, FieldValue, _Ctx, ['$in',
		array([ string(ArgValue), string(FieldValue) ])]) :-
	atom_concat(_,'*',FieldValue),!.
	
triple_arg_value(Arg, ArgValue, FieldValue, _Ctx, [ArgOperator,
		array([ string(FieldValue), string(ArgValue) ])]) :-
	mng_strip_variable(Arg, X),
	mng_strip_operator(X, Operator1, _),
	mng_operator(Operator1, ArgOperator).

%%
graph_doc('any', _) :- !, fail.
graph_doc('*', _)   :- !, fail.
graph_doc(=(GraphName), ['graph',string(GraphName)]) :- !.
graph_doc(  GraphName,  ['graph',['$in',array(Graphs)]]) :-
	ground(GraphName),!,
	findall(string(X),
		(X=GraphName ; sw_graph_includes(GraphName,X)),
		Graphs).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% helper
%%%%%%%%%%%%%%%%%%%%%%%

%%
triple_graph_for_assert(Ctx, Graph) :-
	once((sw_default_graph(DefaultGraph) ; DefaultGraph=user)),
	option(graph(Graph), Ctx, DefaultGraph).

triple_graph_for_query(Ctx, Graph) :-
	option(graph(Graph), Ctx, any).

%%
extend_context(triple(_,P,_), P1, Context, Context0) :-
	% get the collection name
	(	option(collection(Coll), Context)
	;	mongolog_get_db(_DB, Coll, 'triples')
	),
	% read options from argument terms
	% e.g. properties can be wrapped in transitive/1 term to
	% indicate that the property is transitive which will then
	% generate an additional step in the aggregate pipeline.
	strip_property_modifier(P,P_opts,P1),
	% extend the context
	% NOTE: do not use findall here to keep var references in Context valid
	bagof(Opt,
		(	Opt=property(P1)
		;	Opt=collection(Coll)
		;	member(Opt, P_opts)
		;	member(Opt, Context)
		),
		Context0).

%%
get_triple_vars(S, P, O, Ctx, Vars) :-
	findall([Key,Field],
		(	member([Field,Arg], [[s,S],[p,P],[o,O]]),
			mongolog:goal_var(Arg, Ctx, [Key, _Var])
		),
		Vars).

%%
set_triple_vars(S, P, O, Ctx, ['$set', ProjectDoc]) :-
	get_triple_vars(S,P,O,Ctx,TripleVars),
	findall([Key, string(NextValue)],
		(	member([Key, Field], TripleVars),
			atom_concat('$next.', Field, NextValue)
		),
		ProjectDoc),
	ProjectDoc \= [].

%%
strip_property_modifier(Var,[],Var) :- var(Var), !.
strip_property_modifier(Term,[X|Xs],Stripped) :-
	strip_property_modifier1(Term,X,Term0), !,
	strip_property_modifier(Term0,Xs,Stripped).
strip_property_modifier(Stripped,[],Stripped).

strip_property_modifier1(transitive(X),      ostar,      X) :- taxonomical_property(X),!.
strip_property_modifier1(transitive(X),      transitive, X).
strip_property_modifier1(reflexive(X),       reflexive,  X).
strip_property_modifier1(include_parents(X), pstar,      X).

%% is_unique_name(+Name) is semidet.
%
% True if Name is not the subject of any known fact.
%
is_unique_name(Name) :-
	mongolog_get_db(DB, Coll, 'triples'),
	\+ mng_find(DB, Coll, [['s',string(Name)]], _).

%% get_unique_name(+Prefix, -Name) is semidet.
%
% Generates a unique name with given prefix.
%
get_unique_name(Prefix, Name) :-
	% generate 8 random alphabetic characters
	randseq(8, 25, Seq_random),
	maplist(plus(65), Seq_random, Alpha_random),
	atom_codes(Sub, Alpha_random),
	atomic_list_concat([Prefix,'_',Sub], IRI),
	% check if there is no triple with this identifier as subject or object yet
	(	is_unique_name(IRI)
	->	Name=IRI
	;	unique_name(Prefix,Name)
	).
