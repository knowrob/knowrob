:- module(lang_atoms, []).

:- use_module(library('lang/compiler')).

%% query commands
:- query_compiler:add_command(atom_number,         [ask]).
:- query_compiler:add_command(atom_length,         [ask]).
:- query_compiler:add_command(atom_prefix,         [ask]).
:- query_compiler:add_command(atom_concat,         [ask]).
:- query_compiler:add_command(atomic_list_concat,  [ask]).

%% query variables
query_compiler:step_var(atom_number(A,N), Var)      :- query_compiler:get_var([A,N],Var).
query_compiler:step_var(atom_length(A,L), Var)      :- query_compiler:get_var([A,L],Var).
query_compiler:step_var(atom_prefix(A,P), Var)      :- query_compiler:get_var([A,P],Var).
query_compiler:step_var(atom_concat(A1,A2,A3), Var) :- query_compiler:get_var([A1,A2,A3],Var).
query_compiler:step_var(atomic_list_concat(List,Sep,A), Var) :- query_compiler:get_var([A,Sep|List],Var).
query_compiler:step_var(atomic_list_concat(List,A), Var)     :- query_compiler:get_var([A|List],Var).

%% query compilation
query_compiler:step_compile(
		atom_number(Atom,Number), _,
		Pipeline) :-
	query_compiler:var_key_or_val(Atom,Atom0),
	query_compiler:var_key_or_val(Number,Number0),
	findall(Step,
		(	query_compiler:set_if_var(Atom,    ['$toString', Number0], Step)
		;	query_compiler:set_if_var(Number,  ['$toDouble', Atom0],   Step)
		;	query_compiler:match_equals(Atom0, ['$toString', Number0], Step)
		),
		Pipeline).

query_compiler:step_compile(
		atom_length(Atom,Length), _,
		Pipeline) :-
	query_compiler:var_key_or_val(Atom,Atom0),
	query_compiler:var_key_or_val(Length,Length0),
	findall(Step,
		(	query_compiler:set_if_var(Length,  ['$strLenCP', Atom0],   Step)
		;	query_compiler:match_equals(Atom0, ['$toString', Length0], Step)
		),
		Pipeline).

query_compiler:step_compile(
		atom_prefix(Atom,Prefix), _,
		[Step]) :-
	% FIXME: SWI Prolog allows atom(Atom), var(Prefix), and then
	%         yields all possible prefixes.
	query_compiler:var_key_or_val(Atom,Atom0),
	query_compiler:var_key_or_val(Prefix,Prefix0),
	query_compiler:match_equals(Prefix0,
		['$substr', array([
			Atom0, int(0),
			['$strLenCP', Prefix0]
		])],
		Step).

query_compiler:step_compile(
		atom_concat(Left,Right,Atom), _,
		Pipeline) :-
	% FIXME: SWI Prolog allows var(Left), var(Right), atom(Atom), and then
	%         yields all possible concatenations.
	query_compiler:var_key_or_val(Left,Left0),
	query_compiler:var_key_or_val(Right,Right0),
	query_compiler:var_key_or_val(Atom,Atom0),
	findall(Step,
		(	query_compiler:set_if_var(Left, ['$substr', array([Atom0,
				int(0),
				['$subtract', array([ ['$strLenCP',Atom0], ['$strLenCP',Right0] ])]
			])], Step)
		;	query_compiler:set_if_var(Right, ['$substr', array([Atom0,
				['$strLenCP',Left0],
				['$strLenCP',Atom0]
			])], Step)
		;	query_compiler:set_if_var(Atom,    ['$concat', array([Left0,Right0])], Step)
		;	query_compiler:match_equals(Atom0, ['$concat', array([Left0,Right0])], Step)
		),
		Pipeline).

query_compiler:step_compile(
		atomic_list_concat(List, Atom), _,
		Pipeline) :-
	maplist(query_compiler:var_key_or_val, List, List0),
	query_compiler:var_key_or_val(Atom, Atom0),
	findall(Step,
		(	query_compiler:set_if_var(Atom0,   ['$concat', array(List0)], Step)
		;	query_compiler:match_equals(Atom0, ['$concat', array(List0)], Step)
		),
		Pipeline).

query_compiler:step_compile(
		atomic_list_concat(List, Sep, Atom), _,
		Pipeline) :-
	maplist(query_compiler:var_key_or_val, List, List0),
	query_compiler:var_key_or_val(Sep, Sep0),
	query_compiler:var_key_or_val(Atom, Atom0),
	add_separator(List0, Sep0, List1),
	findall(Step,
		(	query_compiler:set_if_var(Atom,    ['$concat', array(List1)], Step)
		;	query_compiler:match_equals(Atom0, ['$concat', array(List1)], Step)
		),
		Pipeline).

%%
add_separator([], _, []) :- !.
add_separator([X], _, [X]) :- !.
add_separator([X0,X1|Xs], Sep, [X0,Sep,X1|Ys]) :-
	add_separator(Xs, Sep, Ys).
