:- module(mng_compiler,
    [ mng_compile/3
    ]).
/** <module> Compile mongo aggregate queries from KnowRob language expresssions.

KnowRob language expressions must be expanded into terminal symbols before.
A sequence of terminals can be compiled into an aggregate query.

@author Daniel Beßler
@license BSD
*/

%% implemented by query commands to compile query documents
:- multifile step_compile/3.
%% implemented by query commands to provide variables exposed to the outside
:- multifile step_var/2.
%% optionally implemented by query commands that require documents from some named collection
:- multifile step_collection/2.

%% mng_compile(+Terminals, -Pipeline, +Context) is semidet.
%
% Compile an aggregate pipeline given a list of terminal symbols
% and the context in which they shall hold.
%
% @Terminals list of terminal symbols
% @Pipeline a term pipeline(Doc,Vars)
% @Context the query context
%
mng_compile(Terminals, pipeline(Doc, Vars), Context) :-
	once(mark_first_(Terminals,Terminals0)),
	catch(
		compile_1(Terminals0, Doc, []->Vars, Context),
		compilation_failed(FailedTerm),
		(	log_error(mongo(compilation_failed(FailedTerm,Terminals))),
			fail
		)
	).

%%
compile_1([], [], V0->V0, _) :- !.
compile_1([X|Xs], Pipeline, V0->Vn, Context) :-
	compile_2(X,  Pipeline_x,  V0->V1, Context),
	compile_1(Xs, Pipeline_xs, V1->Vn, Context),
	% TODO: avoid append
	append(Pipeline_x, Pipeline_xs, Pipeline).

%% Compile a single command (Term) into an aggregate pipeline (Doc).
compile_2(step(Term,Modifier), Doc, V0->V1, Context) :-
	% read all variables referred to in Step into list StepVars
	findall(Vs, step_var(Term, Vs), StepVars),
	% merge StepVars with variables in previous steps (V0)
	append(V0, StepVars, Vars_new),
	list_to_set(Vars_new, V1),
	% compile JSON document for this step
	append(Modifier, Context, InnerContext),
	(	step_compile(Term, Doc, [
				step_vars(StepVars),
				outer_vars(V0) |
				InnerContext
		])
	->	true
	;	throw(compilation_failed(Term, InnerContext))
	).

% mark first triple in list.
% this is needed because the first triple needs special treatment
% in aggregate queries.
mark_first_(
	[step(X,Opt0)|Xs]
	[step(X,[first|Opt0])|Xs]) :-
	(	option(collection(_Coll), Opt0)
	;	step_collection(X,_Coll)
	),
	!.
mark_first_([X|Xs],[X|Ys]) :-
	mark_first_(Xs,Ys).

%% compiler needs to know the first collection from which documents are drawn
first_collection([step(X,Opts)|Xs], Coll) :-
	(	option(collection(Coll), Opts)
	;	step_collection(X,Coll)
	;	first_collection(Xs,Coll)
	),
	!.

first_collection([], Coll) :-
	triple_db(_DB, Coll).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% VARIABLES in queries
%%%%%%%%%%%%%%%%%%%%%%%

%%
% Map a Prolog variable to the key that used to
% refer to this variable in mongo queries.
%
var_key(Var,Key) :-
	var(Var),
	term_to_atom(Var,Atom),
	atom_concat('v',Atom,Key).

%%
% yield either the key of a variable in mongo,
% or a typed term for some constant value provided
% in the query.
%
var_key_or_val(In,Out) :-
	var_key(In,Out),!.

var_key_or_val(In,Out) :-
	atomic(In),!,
	once(get_constant_(In,Out)).

var_key_or_val(In,array(L)) :-
	is_list(In),!,
	findall(X,
		(	member(Y,In),
			var_key_or_val(Y,X)
		),
		L).

var_key_or_val(In,In) :-
	compound(In).

%% in case of atomic in query
get_constant_(Value, double(Value)) :- number(Value).
get_constant_(true,  bool(true)).
get_constant_(false, bool(false)).
get_constant_(Value, string(Value)) :- atom(Value).
get_constant_(Value, string(Value)) :- string(Value).

