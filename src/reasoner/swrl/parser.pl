:- module(swrl_parser,
    [ swrl_file_parse/3,
      swrl_file_fire/1,
      swrl_file_fire/2,
      swrl_file_load/1,
      swrl_file_load/2,
      swrl_file_unload/1,
      swrl_file_unload/2,
      swrl_phrase/3,
      swrl_phrase_fire/2,
      swrl_phrase_assert/2
    ]).
/** <module> Prolog-based SWRL representation.

@author Daniel Beßler
*/

:- use_module(library('dcg/basics')).
:- use_module(library('semweb/rdf_db'), [ rdf_split_url/3 ]).
:- use_module(library('logging')).
:- use_module('swrl').

:- dynamic swrl_file_store/3,
           swrl_assertion_store/3,
           swrl_iri/2.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
swrl_file_fire(Filepath,Label) :-
	swrl_file_parse(Filepath,Rule,Args),
	get_dict(label,Args,Label),!,
	swrl_fire(Rule,Label).

%%
swrl_file_fire(Filepath) :-
	forall(
		swrl_file_parse(Filepath,Rule,Args),
		(	get_dict(label,Args,Label),
			swrl_fire(Rule,Label)
		)
	).

%% swrl_file_unload(+Filepath) is det.
%% swrl_file_unload(+Filepath,+Label) is det.
%
% Unload previously loaded SWRL file,
% retracting all the rules that have been asserted
% so far.
%
swrl_file_unload(Filepath) :-
  forall( swrl_file_store(Filepath,Rule,_), (
      retract( swrl_file_store(Filepath,Rule,_) ),
      ignore(retract( Rule ))
  )).

swrl_file_unload(Filepath,Label) :-
  forall(
    ( swrl_file_store(Filepath, Rule, Args),
      get_dict(label,Args,Label) ),
    ( retract( swrl_file_store(Filepath,Rule) ),
      ignore(retract( Rule ))
  )).

%% swrl_file_load(+Filepath) is det.
%% swrl_file_load(+Filepath,+Label) is det.
%
% Loads all or one rule(s) from a SWRL file.
%
swrl_file_load(Filepath,Label) :-
    swrl_assertion_store(Filepath, _Rule, Args),
    get_dict(label,Args,Label),
    !.

swrl_file_load(Filepath,Label) :-
    swrl_file_parse(Filepath,Rule,Args),
    get_dict(label,Args,Label),!,
    swrl_assert_rule(Rule),
    assertz(swrl_assertion_store(Filepath, _Rule, Args)).

swrl_file_load(Filepath) :-
    forall(
        (   swrl_file_parse(Filepath,Rule,Args),
            \+ swrl_assertion_store(Filepath,Rule,Args)
        ),
        (   swrl_assert_rule(Rule),
            assertz(swrl_assertion_store(Filepath,Rule,Args))
        )).

%% swrl_file_parse(+Filepath,-Rule,-Args) is det.
%
% Parses a SWRL file.
% 
swrl_file_parse(FileSpec,Rule,Args) :-
    swrl_file_store(FileSpec,Rule,Args) *-> true ; (
        absolute_file_name(FileSpec, FilePath, []),
        phrase_from_file(swrl_file_rules(_,Xs), FilePath),
        findall([R,A],
            (   member([Pairs,R],Xs),
                dict_pairs(A,_,Pairs),
                assertz(swrl_file_store(FileSpec,R,A))
            ), Rules),
        member([Rule,Args],Rules)
    ).
  
%%
swrl_file_rules(_,[])            --> call(eos), !.
swrl_file_rules(NS,Rules)        --> swrl_file_ns(NS), !, swrl_file_rules(NS,Rules).
swrl_file_rules(NS,[Rule|Rules]) --> swrl_file_rule(NS,Rule), swrl_file_rules(NS,Rules).

%%
swrl_file_ns(NS) -->
  ":-", blanks, "namespace('", string(NSCodes), "')", ".",
  { atom_codes(NS, NSCodes) }.

%%
swrl_file_rule(_,[])        --> call(eos), !.
swrl_file_rule(NS,[Args,Rule]) --> ":-", swrl_file_args(Args), ",", !, swrl_file_rule_(NS,Rule).
swrl_file_rule(NS,[[],Rule])   --> ":-", swrl_file_rule_(NS,Rule), !.
swrl_file_rule(NS,Rule)        --> [_], swrl_file_rule(NS,Rule).
swrl_file_rule_(NS,RuleTerm) -->
  blanks, string(RulesCodes), ".",
  { atom_codes(RuleAtom, RulesCodes),!,
    swrl_phrase(RuleTerm, RuleAtom, NS) }, !.

%%
swrl_file_args(Args) --> blanks, "{", !, swrl_file_args_(Args), blanks.
swrl_file_args([])   --> "".
swrl_file_args_([Key-Value|Xs]) -->
  blanks, ("'";""), string(KeyCodes), ("'";""), 
  blanks, ":",
  blanks, ("'";""), string(ValueCodes), ("'";""), 
  blanks,
  ( (",", swrl_file_args_(Xs)) ;
    ("}", {Xs=[]}) ), !,
  { atom_codes(Key, KeyCodes),
    atom_codes(Value, ValueCodes)
  }.
swrl_file_args_([]) --> "".

%% swrl_phrase_fire(+Phrase,+NS).
%
% Fires SWRL rule given in human readable Syntax.
%
swrl_phrase_fire(Phrase,NS) :-
  swrl_phrase(Term, Phrase, NS),
  swrl_fire(Term).

%% swrl_phrase_assert(+Phrase,+NS).
%
% Assert SWRL rule in human readable Syntax as native Prolog rule(s).
%
swrl_phrase_assert(Phrase,NS) :-
  swrl_phrase(Term, Phrase, NS),
  swrl_assert_rule(Term).

%% swrl_phrase(?Term, ?Expr).
%
% `Term` is is the parse tree of parsing the atom `Expr` as SWRL rule.
% Can be used to generate human readable syntax for Prolog SWRL terms,
% but also for generating Prolog term representation of human readable
% SWRL expression.
%
% For example, `swrl_phrase(Term, 'Driver(?p) -> Person(?p)')`
% parses the expression and generates the term:
% 
%   Term = [ class(test_swrl:'Person',var(p)) ] :-
%                [ class(test_swrl:'Driver',var(p)) ]
%
% @param Term A Prolog term describing a SWRL rule
% @param Expr A SWRL describtion in human readable syntax
%
swrl_phrase(Term, Expr, NS) :-
  ground(Term),!,
  phrase(swrl_parser(Term, NS), Tokens),
  swrl_tokens_format(Tokens, Expr).

swrl_phrase(Term, Expr, NS) :-
  atom(Expr),!,
  tokenize_atom(Expr, Tokens),
  phrase(swrl_parser(Term, NS), Tokens).

swrl_tokens_format(Tokens, Expr) :-
  swrl_token_spaces(Tokens, Spaces),
  atomic_list_concat(Spaces,'',Expr).

swrl_token_spaces([],[]).
swrl_token_spaces(In, Out) :-
  swrl_token_spaces(In, Head, Rest),
  swrl_token_spaces(Rest, Tail),
  append(Head,Tail,Out).

swrl_token_spaces(['-','>'|Xs],[' -> '],Xs) :- !.
swrl_token_spaces([Op|Xs], [' ',Op,' '],Xs)         :- member(Op, ['or','and','value','some','all']), !.
swrl_token_spaces([Op|Xs], [Op,' '],Xs)             :- member(Op, ['not',',']), !.
swrl_token_spaces([Op,N|Xs], [' ',Op,' ',N,' '],Xs) :- member(Op, ['exactly','min','max']), !.
swrl_token_spaces([X|Xs], [X],Xs).

swrl_parser([],_)     --> [].
swrl_parser(Tree, NS) --> swrl_rule(Tree, NS).

swrl_rule(Head :- Body, NS)      --> swrl_conjunction(Body,NS), ['-'], ['>'], swrl_conjunction(Head,NS). 
swrl_conjunction([Atom|Rest],NS) --> swrl_literal(Atom,NS), [','], swrl_conjunction(Rest,NS).
swrl_conjunction([Atom],NS)      --> swrl_literal(Atom,NS).

%%
swrl_literal(class(A,B),NS) -->
  swrl_class_atom(A,NS), ['('], swrl_subject(B,NS), [')'].

swrl_literal(property(S,P,O),NS) -->
  swrl_property(P,NS), ['('], swrl_subject(S,NS), [','], swrl_data_object(O), [')'],
  { swrl:swrl_data_property(P) }.

swrl_literal(property(S,P,O),NS) -->
  swrl_property(P,NS), ['('], swrl_subject(S,NS), [','], swrl_object(O,NS), [')'],
  { swrl:swrl_object_property(P) }.

swrl_literal(BuiltinTerm,_NS) -->
  { (ground(BuiltinTerm), BuiltinTerm =.. [Predicate|Args]) ; true },
  swrl_builtin(Predicate), ['('], swrl_builtin_args(Args), [')'],
  { BuiltinTerm =.. [Predicate|Args] }.

%%
swrl_class_atom(complement_of(Cls),NS) -->
  ['('], ['not'], swrl_class_atom(Cls,NS), [')'].

swrl_class_atom(intersection_of([X|Xs]),NS) -->
  ['('], swrl_class_atom_terminal(X,NS), ['and'], swrl_class_intersection(Xs,NS), [')'].

swrl_class_atom(union_of([X|Xs]),NS) -->
  ['('], swrl_class_atom_terminal(X,NS), ['or'], swrl_class_union(Xs,NS), [')'].

swrl_class_atom(some(P,Cls),NS) -->
  ['('], swrl_property(P,NS), ['some'], swrl_class_atom(Cls,NS), [')'].

swrl_class_atom(only(P,Cls),NS) -->
  ['('], swrl_property(P,NS), ['all'],  swrl_class_atom(Cls,NS), [')'].

swrl_class_atom(value(P,Value),NS) -->
  ['('], swrl_property(P,NS), ['value'], swrl_value(Value,NS), [')'].

swrl_class_atom(exactly(Num,P,Cls),NS) -->
  ['('], swrl_property(P,NS), ['exactly'], swrl_number(Num), swrl_class_atom_terminal(Cls,NS), [')'].

swrl_class_atom(max(P,Num,Cls),NS) -->
  ['('], swrl_property(P,NS), ['max'], swrl_number(Num), swrl_class_atom_terminal(Cls,NS), [')'].

swrl_class_atom(min(P,Num,Cls),NS) -->
  ['('], swrl_property(P,NS), ['min'], swrl_number(Num), swrl_class_atom_terminal(Cls,NS), [')'].

swrl_class_atom(Cls,NS) --> swrl_class_atom_terminal(Cls, NS).

swrl_class_atom_terminal(class(Cls),NS) --> [Cls_name], { swrl_match_instance(Cls,Cls_name,NS) }.

swrl_class_intersection([Cls],NS)      --> swrl_class_atom(Cls,NS).
swrl_class_intersection([Cls|Rest],NS) --> swrl_class_atom(Cls,NS), ['and'], swrl_class_intersection(Rest,NS).

swrl_class_union([Cls],NS)             --> swrl_class_atom(Cls,NS).
swrl_class_union([Cls|Rest],NS)        --> swrl_class_atom(Cls,NS), ['or'], swrl_class_union(Rest,NS).

swrl_subject(S,_NS)   --> swrl_var_expr(S).
swrl_subject(S,NS)    --> swrl_individual(S,NS).

swrl_property(P,NS)   --> [P_name], { swrl_match_instance(P,P_name,NS) }.

swrl_object(Var,_NS)  --> swrl_var_expr(Var).
swrl_object(Obj,NS)   --> swrl_individual(Obj,NS).
swrl_object(Cls,NS)   --> swrl_class_atom_terminal(Cls,NS).

swrl_data_object(Var) --> swrl_var_expr(Var).
swrl_data_object(Val) --> ['"'], [Val], ['"'].
swrl_data_object(Val) --> [Val].

swrl_individual(I,NS) --> [I_name], { swrl_match_instance(I,I_name,NS) }.

swrl_number(Num) --> [Num], { number(Num) }.

swrl_value(Val,NS)   --> swrl_individual(Val,NS).
swrl_value(Val,NS)   --> swrl_class_atom_terminal(Val,NS).
swrl_value(Val,_NS)   --> swrl_data_object(Val).

swrl_builtin_args([Arg|Rest]) --> swrl_builtin_arg(Arg), [','], swrl_builtin_args(Rest).
swrl_builtin_args([Arg])      --> swrl_builtin_arg(Arg).
swrl_builtin_arg(Var)         --> swrl_var_expr(Var).
swrl_builtin_arg(Val)         --> [Val], { number(Val) }.
swrl_builtin_arg(Val)         --> ['"'], [Val], ['"'].
swrl_builtin_arg(Atomic)      --> [Atomic], { atomic(Atomic) }.
swrl_builtin(Predicate)       --> [Predicate], { swrl_is_builtin(Predicate) }.

swrl_var_expr(var(Var)) --> ['?'], [Var].

swrl_is_builtin(Predicate) :-
  atom(Predicate), % check if predicate is a builtin predicate
  clause(swrl:swrl_builtin(Predicate,_,_,_),_).

swrl_match_instance(IRI,Name,_) :-
  atom(IRI),!,
  rdf_split_url(_, Name, IRI).

% TODO: swrl_match_instance called with invalid input a lot, can be avoided?
swrl_match_instance(_,'?',_)     :- !, fail.
swrl_match_instance(_,'(',_)     :- !, fail.
swrl_match_instance(_,'not',_)   :- !, fail.
swrl_match_instance(_,'true',_)  :- !, fail.
swrl_match_instance(_,'false',_) :- !, fail.
swrl_match_instance(_,Name,_)    :- swrl_is_builtin(Name), !, fail.

swrl_match_instance(IRI,Name,_NS) :-
	var(IRI), atom(Name),
	% get cached IRI
	swrl_iri(Name, IRI),
	!.

swrl_match_instance(IRI,Name,NS) :-
	var(IRI), atom(Name), atom(NS),
	% try to use user-specified namespace to find the entity
	atom_concat(NS,Name,IRI),
	% check if IRI is a currently defined IRI
	swrl_subject(IRI),!,
	assertz(swrl_iri(Name,IRI)).

swrl_match_instance(IRI,Name,NS) :-
	throw(error(existence_error(instance,Name),
	            swrl_match_instance(IRI,Name,NS))).
