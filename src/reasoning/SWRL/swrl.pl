:- module(swrl,
		[ swrl_assert/1
		]).
/** <module> Prolog-based SWRL representation.

@author Daniel Beßler
*/

:- use_module(library('lang/terms/holds'),
		[ holds/3 ]).
:- use_module(library('model/OWL')).

%% swrl_assert(+Rule).
%
% Assert SWRL rule in the Prolog KB.
%
% @param Rule Prolog-based representation of SWRL rule.
%
swrl_assert([] :- _) :- !.

swrl_assert([HeadAtom|Xs] :- Body) :-
	swrl_assert1(HeadAtom :- Body),
	swrl_assert(Xs :- Body).

swrl_assert1(HeadAtom :- Body) :-
	swrl_vars([HeadAtom] :- Body, Vars),
	swrl_rule_pl(
		HeadAtom :- Body,
		Rule_pl, [var('swrl:scope',_)|Vars]),
	% TODO need to support facts??
	Rule_pl=(?>(Impl_pl,Cond_pl)),
	comma_list(Cond_pl0,Cond_pl),
	% expand ask rule into regular Prolog rule
	% NOTE: this also calls query_assert/1
	expand_term(?>(Impl_pl,Cond_pl0), Expanded),
	% finally assert another clause of instance_of or holds
	assertz(Expanded).
  
swrl_rule_pl_implication(Implication :- _, Implication) :- !.
swrl_rule_pl_implication(Implication, Implication).

%% swrl_rule_pl
swrl_rule_pl(Fact :- [], Fact_pl, Vars) :-
	!, swrl_implication_pl(Fact, Fact_pl, Vars).

swrl_rule_pl(
		class(Cls,S)                      :- Cond,
		model_RDFS:instance_of(S_var,Cls) ?> Cond_pl,
		Vars) :-
	swrl_var(Vars, S, S_var),
	swrl_condition_pl(Cond, Cond_pl0, Vars),!,
	Cond_pl=[pragma(\+ compound(Cls)) | Cond_pl0].

swrl_rule_pl(
		property(S,P,O)                 :- Cond,
		lang_holds:holds(S_var,P,O_var) ?> Cond_pl,
		Vars) :-
	swrl_var(Vars, S, S_var),
	swrl_var(Vars, O, O_var),
	swrl_condition_pl(Cond, Cond_pl0, Vars),!,
	Cond_pl=[pragma(\+ compound(O_var)) | Cond_pl0].
  
%% swrl_condition_pl
swrl_condition_pl([], [], _) :- !.

swrl_condition_pl([X|Xs], [Y|Ys], Vars) :-
	swrl_condition_pl(X, Y, Vars),
	swrl_condition_pl(Xs, Ys, Vars).
  
swrl_condition_pl(
		class(Cls,S),
		instance_of(S_var,Cls),
		Vars) :-
	swrl_var(Vars, S, S_var).

swrl_condition_pl(
		property(S,P,O),
		holds(S_var,P,O_var),
		Vars) :-
	swrl_var(Vars, S, S_var),
	swrl_var(Vars, O, O_var).

swrl_condition_pl(Builtin, Builtin_pl, Vars) :-
	swrl_builtin_pl(Builtin, Builtin_pl, Vars).

%% swrl_builtin_pl
% TODO (DB): support more builtins: matches, listConcat, member, length, ...
%
swrl_builtin_pl(
		equal(S,O),
		(S_var == O_var),
		Vars) :-
	swrl_var(Vars, S, S_var),
	swrl_var(Vars, O, O_var).

swrl_builtin_pl(
		notEqual(S,O),
		(S_var \== O_var),
		Vars) :-
	swrl_var(Vars, S, S_var),
	swrl_var(Vars, O, O_var).

swrl_builtin_pl(
		lessThan(A,B),
		(A_num < B_num),
		Vars) :-
	swrl_nums([A,B], [A_num,B_num], Vars).

swrl_builtin_pl(
		lessThanOrEqual(A,B),
		(A_num =< B_num),
		Vars) :-
	swrl_nums([A,B], [A_num,B_num], Vars).

swrl_builtin_pl(
		greaterThan(A,B),
		(A_num > B_num),
		Vars) :-
	swrl_nums([A,B], [A_num,B_num], Vars).

swrl_builtin_pl(
		greaterThanOrEqual(A,B),
		(A_num >= B_num),
		Vars) :-
	swrl_nums([A,B], [A_num,B_num], Vars).

%% math builtins
swrl_builtin_pl(
		add(A,B,C),
		(A_num is B_num + C_num),
		Vars) :-
	swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).

swrl_builtin_pl(
		subtract(A,B,C),
		(A_num is B_num - C_num),
		Vars) :-
	swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).

swrl_builtin_pl(
		multiply(A,B,C),
		(A_num is B_num * C_num),
		Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).

swrl_builtin_pl(
		divide(A,B,C),
		(A_num is B_num / C_num),
		Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).

swrl_builtin_pl(
		mod(A,B,C),
		(A_num is B_num mod C_num),
		Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).

swrl_builtin_pl(
		pow(A,B,C),
		(A_num is B_num ** C_num),
		Vars) :-
  swrl_nums([A,B,C], [A_num,B_num,C_num], Vars).

swrl_builtin_pl(
		abs(A,B),
		(A_num is abs(B_num)),
		Vars) :-
  swrl_nums([A,B], [A_num,B_num], Vars).

%% boolean builtins
swrl_builtin_pl(booleanNot(S), not(S_var), Vars) :-
  swrl_var(Vars, S, S_var).

%% string builtins
swrl_builtin_pl(
		stringConcat(A,B,C),
		atom_concat(C_atom,A_atom,B_atom),
		Vars) :-
  swrl_atoms([A,B,C], [A_atom,B_atom,C_atom], Vars).

swrl_builtin_pl(
		stringLength(A,L),
		atom_length(A_atom, L_num),
		Vars) :-
  swrl_atom(A,A_atom,Vars),
  swrl_atom_number(L,L_num,Vars).

swrl_builtin_pl(
		upperCase(A,Upper),
		upcase_atom(A_atom, Upper_atom),
		Vars) :-
  swrl_atoms([A,Upper], [A_atom,Upper_atom], Vars).

swrl_builtin_pl(
		lowerCase(A,Lower),
		downcase_atom(A_atom, Lower_atom),
		Vars) :-
  swrl_atoms([A,Lower], [A_atom,Lower_atom], Vars).

swrl_builtin_pl(
		contains(A,X),
		sub_atom(A_atom, _, _, _, X_atom),
		Vars) :-
  swrl_atoms([A,X], [A_atom,X_atom], Vars).

swrl_builtin_pl(
		startsWith(A,X),
		atom_prefix(A_atom, X_atom),
		Vars) :-
  swrl_atoms([A,X], [A_atom,X_atom], Vars).

swrl_builtin_pl(
		endsWith(A,X),
		atom_concat(_, X_atom, A_atom),
		Vars) :-
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
