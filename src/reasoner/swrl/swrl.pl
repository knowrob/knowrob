:- module(swrl,
		[ swrl_fire/1          % +Rule
		, swrl_fire/2          % +Rule, +Label
		, swrl_assert_rule/1   % +Rule
		, swrl_assert_rule/2   % +Rule, +Label
		, swrl_rule_hash/2     % +Rule, -Hash
		, swrl_subject/1
		, swrl_data_property/1
		, swrl_object_property/1
		]).
/** <module> Prolog-based SWRL representation.

@author Daniel Beßler
*/

:- use_module(library('scope')).
:- use_module(library('semweb')).
:- use_module(library('reasoner')).
:- use_module(library('semweb/rdf_db'), [ rdf_subject/1 ]).
:- use_module(library('semweb/rdfs'), [ rdfs_individual_of/2 ]).

:- multifile swrl_builtin/4.
% define reasoner settings.
:- reasoner_setting(swrl:knowledgeGraph, atom, prolog, 'Must be one of `prolog` or `mongolog`').

%% swrl_call(+Goals) is nondet.
%
% The call is restricted to the current reasoner context.
% Thus results of other reasoners are only incorparated as long
% as they are stored in the data backend that is used by the SWRL reasoner.
%
swrl_call(Goals) :-
    is_list(Goals),!,
    comma_list(Goal,Goals),
    current_reasoner_module(Reasoner),
    Reasoner:call(Goal).

swrl_call(Goal) :-
    throw(error(type_error(goal, Goal), swrl_call(Goal))).

%%
% True when S is a currently known subject.
%
swrl_subject(Resource) :-
    rdf_subject(Resource),
    !.

%%
%
swrl_data_property(P) :-
    rdfs_individual_of(P, owl:'DatatypeProperty'),
    !.

%%
%
swrl_object_property(P) :-
    rdfs_individual_of(P, owl:'ObjectProperty'),
    !.

%% swrl_rule_hash(+Rule, -Hash) is det.
%
% Compute the hash a SWRL rule.
% Internally call term_hash/2.
%
swrl_rule_hash(Rule, Hash) :-
	term_hash(Rule,X),
	atom_number(Hash,X).

%% swrl_fire(+Rule).
%
% Same as swrl_fire/2 but uses the hash of the rule as label.
%
% @param Rule Prolog-based representation of SWRL rule.
%
swrl_fire(Rule) :-
	% generate a label for the rule from its signature
	swrl_rule_hash(Rule,Label),
	swrl_fire(Rule,Label).

%% swrl_fire(+Rule, +Label).
%
% Fires a rule. That is running it over the whole knowledge base
% and asserting inferred facts.
% Label is a unique identifier of the rule used to avoid redundancy.
%
swrl_fire(Head :- Body, Label) :-
	atom_concat('swrl:', Label, Label0),
	forall(member(HeadAtom,Head),
		   swrl_fire1(HeadAtom :- Body, Label0)).

%
swrl_fire1(SWRLRule, Label) :-
	% translate into a Prolog rule
	swrl_rule_pl_(SWRLRule, Label, (:-(Impl_pl,Cond_pl))),
	forall(swrl_call(Cond_pl), swrl_assert_fact(Impl_pl)).

%% swrl_assert_rule(+Rule).
%
% Same as swrl_assert/2 but uses the hash of the rule as label.
%
% @param Rule Prolog-based representation of SWRL rule.
%
swrl_assert_rule(Rule) :-
	% generate a label for the rule from its signature
	swrl_rule_hash(Rule,Label),
	swrl_assert_rule(Rule,Label).

%% swrl_assert_rule(+Rule, +Label).
%
% Assert SWRL rule in the knowledge base.
% Label is a unique identifier of the rule used to avoid redundancy.
%
swrl_assert_rule(Head :- Body, Label) :-
	atom_concat('swrl:', Label, Label0),
	forall(member(HeadAtom,Head),
		   swrl_assert_rule1(HeadAtom :- Body, Label0)).

%
swrl_assert_rule1(SWRLRule, Label) :-
	swrl_rule_pl_(SWRLRule, Label, (:-(Impl_pl,Cond_pl))),
	comma_list(Cond_pl0,Cond_pl),
	% term expansion
	expand_term(:-(Impl_pl,Cond_pl0), Expanded),
	% finally assert expanded rule
	swrl_assert_rule2(Expanded).

%%
swrl_assert_rule2(Rule) :-
	current_reasoner_module(ReasonerModule),
	ReasonerModule:assertz(Rule).

%% swrl_assert_fact(+Term) is semidet.
%
swrl_assert_fact(instance_of(S,class(Cls))) :-
    !,
    swrl_assert_fact(instance_of(S,Cls)).
swrl_assert_fact(instance_of(S,Cls)) :-
    (atom(Cls);compound(Cls)),!,
    sw_assert_type(S,Cls).

swrl_assert_fact(triple(S,P,O)) :-
    atom(S), atom(P), ground(O),!,
    sw_assert_triple(S,P,O).

swrl_assert_fact(Predicate) :-
    throw(error(type_error(predicate, Predicate), swrl_assert_fact(Predicate))).

%% swrl_rule_pl
swrl_rule_pl(Fact :- [], Fact_pl, Vars) :-
	!,
	swrl_implication_pl(Fact, Fact_pl, Vars).

swrl_rule_pl(class(class(Cls),S)     :- Cond,
		     instance_of(S_var,Cls)  :- Cond_pl, Vars) :-
	swrl_var(Vars, S, S_var),
	swrl_condition_pl(Cond, Cond_pl, Vars),!.

swrl_rule_pl(class(Cls,S)           :- Cond,
		     instance_of(S_var,Cls) :- Cond_pl, Vars) :-
	swrl_var(Vars, S, S_var),
	swrl_condition_pl(Cond, Cond_pl, Vars),!.

swrl_rule_pl(property(S,P,O)       :- Cond,
		     triple(S_var,P,O_var) :- Cond_pl, Vars) :-
	swrl_var(Vars, S, S_var),
	swrl_var(Vars, O, O_var),
	swrl_condition_pl(Cond, Cond_pl, Vars),!.

%%
swrl_rule_pl_(HeadAtom :- Body, Label, Rule_pl) :-
	% parse rule variables into a map
	swrl_vars([HeadAtom] :- Body, Vars),
	RuleVars = [var('swrl:scope',_), var('swrl:label',Label) | Vars],
	% translate into a Prolog rule
	swrl_rule_pl(HeadAtom :- Body, Rule_pl, RuleVars).
  
%% swrl_condition_pl
swrl_condition_pl([], [], _) :- !.

swrl_condition_pl([X|Xs], [Y|Ys], Vars) :-
	swrl_condition_pl(X, Y, Vars),
	swrl_condition_pl(Xs, Ys, Vars).
  
swrl_condition_pl(class(Cls,S),
		          instance_of_expr(S_var,Cls), Vars) :-
	compound(Cls),!,
	swrl_var(Vars, S, S_var).

swrl_condition_pl(class(Cls,S),
		          instance_of(S_var,Cls), Vars) :-
	swrl_var(Vars, S, S_var).

swrl_condition_pl(property(S,P,O),
		          triple(S_var,P,O_var), Vars) :-
	swrl_var(Vars, S, S_var),
	swrl_var(Vars, O, O_var).

swrl_condition_pl(Builtin, Builtin_pl, Vars) :-
	compound(Builtin),
	Builtin =.. [Functor|Args],
	swrl_builtin(Functor, Args, Builtin_pl, Vars).

%% swrl_builtin
%
swrl_builtin(equal, [S,O],
		    (S_var == O_var), Vars) :-
	swrl_var(Vars, S, S_var),
	swrl_var(Vars, O, O_var).

swrl_builtin(notEqual, [S,O],
		    (S_var \== O_var), Vars) :-
	swrl_var(Vars, S, S_var),
	swrl_var(Vars, O, O_var).

swrl_builtin(lessThan, [A,B],
		    (A_num < B_num), Vars) :-
	swrl_nums([A,B], [A_num,B_num], Vars).

swrl_builtin(lessThanOrEqual, [A,B],
		    (A_num =< B_num), Vars) :-
	swrl_nums([A,B], [A_num,B_num], Vars).

swrl_builtin(greaterThan, [A,B],
		    (A_num > B_num), Vars) :-
	swrl_nums([A,B], [A_num,B_num], Vars).

swrl_builtin(greaterThanOrEqual, [A,B],
		    (A_num >= B_num), Vars) :-
	swrl_nums([A,B], [A_num,B_num], Vars).

%% math builtins
swrl_builtin(add, [A,B,C],
		    (A_num is B_num + C_num), Vars) :-
	swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).

swrl_builtin(subtract, [A,B,C],
		    (A_num is B_num - C_num), Vars) :-
	swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).

swrl_builtin(multiply, [A,B,C],
		    (A_num is B_num * C_num), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).

swrl_builtin(divide, [A,B,C],
            (A_num is B_num / C_num), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).

swrl_builtin(mod, [A,B,C],
            (A_num is B_num mod C_num), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).

swrl_builtin(pow, [A,B,C],
            (A_num is B_num ** C_num), Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).

swrl_builtin(abs, [A,B],
            (A_num is abs(B_num)), Vars) :-
  swrl_nums([A,B], [A_num,B_num], Vars).

%% boolean builtins
swrl_builtin(booleanNot, [S], not(S_var), Vars) :-
  swrl_var(Vars, S, S_var).

%% string builtins
swrl_builtin(stringConcat, [A,B,C],
             atom_concat(C_atom,A_atom,B_atom), Vars) :-
  swrl_atoms([A,B,C], [A_atom,B_atom,C_atom], Vars).

swrl_builtin(stringLength, [A,L],
             atom_length(A_atom, L_num), Vars) :-
  swrl_atom(A,A_atom,Vars),
  swrl_atom_number(L,L_num,Vars).

swrl_builtin(upperCase, [A,Upper],
             upcase_atom(A_atom, Upper_atom), Vars) :-
  swrl_atoms([A,Upper], [A_atom,Upper_atom], Vars).

swrl_builtin(lowerCase, [A,Lower],
             downcase_atom(A_atom, Lower_atom), Vars) :-
  swrl_atoms([A,Lower], [A_atom,Lower_atom], Vars).

swrl_builtin(contains, [A,X],
             sub_atom(A_atom, _, _, _, X_atom), Vars) :-
  swrl_atoms([A,X], [A_atom,X_atom], Vars).

swrl_builtin(startsWith, [A,X],
             atom_prefix(A_atom, X_atom), Vars) :-
  swrl_atoms([A,X], [A_atom,X_atom], Vars).

swrl_builtin(endsWith, [A,X],
             atom_concat(_, X_atom, A_atom), Vars) :-
  swrl_atoms([A,X], [A_atom,X_atom], Vars).


%% Find all the variables in a SWRL rule and map those to anonymous
%% Prolog variables for later unification.
swrl_vars(Head :- Body, Variables) :-
	findall(VarName,
		(	(member(Atom,Head) ; member(Atom,Body)),
			swrl_var_names(Atom,VarName)
		),
		VarNames),
	list_to_set(VarNames, VarNamesUnique),
	maplist(swrl_var_pl(_), VarNamesUnique,Variables).
swrl_var_pl(_,Y,var(Y,_)).

%% Find variables in a SWRL atom
swrl_var_names(var(Var), Var) :- atom(Var), !.
swrl_var_names(Term, Var) :-
	compound(Term), !,
	Term =.. [_|Args],
	member(Arg, Args),
	swrl_var_names(Arg, Var).

%% Map between name of a SWRL variable and the corresponding
%% Prolog variable.
swrl_var(Vars, var(X), PrologVar) :-
	!, swrl_var(Vars, X, PrologVar).
swrl_var([var(Name,PrologVar)|_], Name, PrologVar) :- !.
swrl_var([_|Rest], Name, PrologVar) :-
	swrl_var(Rest, Name, PrologVar).
swrl_var([], Constant, Typed) :-
	mng_client:mng_strip_type(Constant,Type,X),
	mng_client:mng_strip_type(Typed,Type,X).

%%
swrl_atom(A, B, _) :-
	var(A),!, B=A.
swrl_atom(var(A), A_val, Vars) :-
	swrl_var(Vars, A, A_var),
	swrl_atom(A_var,A_val,Vars).
swrl_atom(Atom, Atom, _) :-
	atom(Atom).
swrl_atoms([], [], _).
swrl_atoms([X|Xs], [Y|Ys], Vars) :-
	swrl_atom(X,Y,Vars),
	swrl_atoms(Xs,Ys,Vars).

%%
swrl_atom_number(var(A), A_num, Vars) :-
	!, swrl_var(Vars, A, A_var),
	(	ground(A_var)
	->	swrl_atom_number(A_var, A_num, Vars)
	;	A_num = A_var
	).

swrl_atom_number(A, A_num, _) :-
	atom(A), !,
	catch(atom_number(A,A_num), _, fail).

swrl_atom_number(A_num, A_num, _) :-
	number(A_num), !.

%%
swrl_nums([],[],_).
swrl_nums([X|Xs],[Y|Ys],Vars) :-
	swrl_atom_number(X,Y,Vars),
	swrl_nums(Xs,Ys,Vars).
