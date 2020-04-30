
:- module(swrl_parser,
    [
      swrl_file_path/3,
      swrl_file_parse/3,
      swrl_file_load/1,
      swrl_file_load/2,
      swrl_file_unload/1,
      swrl_file_unload/2,
      swrl_phrase/2,
      swrl_phrase_assert/1
    ]).
/** <module> Prolog-based SWRL representation.

@author Daniel Beßler
*/

:- use_module(library('dcg/basics')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('swrl')).

:- dynamic swrl_file_store/3,
           swrl_assertion_store/3.

%%
swrl_file_path(Pkg,Filename,Filepath) :-
  ros_package_path(Pkg,PkgPath),
  atomic_list_concat(
    [PkgPath,'swrl',Filename],
    '/',
    Filepath
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
  get_dict(label,Args,Label),!.

swrl_file_load(Filepath,Label) :-
  swrl_file_parse(Filepath,Rule,Args),
  get_dict(label,Args,Label),!,
  swrl_assert(Rule),
  assertz(swrl_assertion_store(Filepath, _Rule, Args)).

swrl_file_load(Filepath) :-
  forall(
  ( swrl_file_parse(Filepath,Rule,Args),
    \+ swrl_assertion_store(Filepath,Rule,Args) ),
  ( swrl_assert(Rule),
    assertz(swrl_assertion_store(Filepath,Rule,Args))
  )).

%% swrl_file_parse(+Filepath,-Rule,-Args) is det.
%
% Parses a SWRL file.
% 
swrl_file_parse(Filepath,Rule,Args) :-
  swrl_file_store(Filepath,Rule,Args) *-> true ; (
    phrase_from_file(swrl_file_rules(Xs), Filepath),
    findall([R,A], (
      member([Pairs,R],Xs),
      dict_pairs(A,_,Pairs),
      assertz(swrl_file_store(Filepath,R,A))
    ), Rules),
    member([Rule,Args],Rules)
  ).
  
%%
swrl_file_rules([])           --> call(eos), !.
swrl_file_rules([Rule|Rules]) --> swrl_file_rule(Rule), swrl_file_rules(Rules).

%%
swrl_file_rule([])          --> call(eos), !.
swrl_file_rule([Args,Rule]) --> ":-", swrl_file_args(Args), ",", swrl_file_rule_(Rule), !.
swrl_file_rule([[],Rule])   --> ":-", swrl_file_rule_(Rule), !.
swrl_file_rule(Rule)        --> [_], swrl_file_rule(Rule).
swrl_file_rule_(RuleTerm) -->
  blanks, string(RulesCodes), ".",
  { atom_codes(RuleAtom, RulesCodes),
    swrl_phrase(RuleTerm, RuleAtom) }, !.

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
swrl_phrase(Term, Expr) :-
  ground(Term),
  phrase(swrl_parser(Term), Tokens),
  swrl_tokens_format(Tokens, Expr).
swrl_phrase(Term, Expr) :-
  atom(Expr),
  tokenize_atom(Expr, Tokens),
  phrase(swrl_parser(Term), Tokens).

%% swrl_phrase_assert(?Phrase).
%
% Assert SWRL rule in human readable Syntax as native Prolog rule(s).
%
swrl_phrase_assert(Phrase) :-
  swrl_phrase(Term, Phrase),
  swrl_assert(Term).

swrl_tokens_format(Tokens, Expr) :-
  swrl_token_spaces(Tokens, Spaces),
  atomic_list_concat(Spaces,'',Expr).

swrl_token_spaces([],[]).
swrl_token_spaces(In, Out) :-
  swrl_token_spaces(In, Head, Rest),
  swrl_token_spaces(Rest, Tail),
  append(Head,Tail,Out).

swrl_token_spaces(['-','>'|Xs],[' -> '],Xs).
swrl_token_spaces([Op|Xs], [' ',Op,' '],Xs)         :- member(Op, ['or','and','value','some','all']).
swrl_token_spaces([Op|Xs], [Op,' '],Xs)             :- member(Op, ['not',',']).
swrl_token_spaces([Op,N|Xs], [' ',Op,' ',N,' '],Xs) :- member(Op, ['exactly','min','max']).
swrl_token_spaces([X|Xs], [X],Xs).

swrl_parser([])   --> [].
swrl_parser(Tree) --> swrl_rule(Tree).

swrl_rule(Head :- Body)       --> swrl_conjunction(Body), ['-'], ['>'], swrl_conjunction(Head). 
swrl_conjunction([Atom|Rest]) --> swrl_literal(Atom), [','], swrl_conjunction(Rest).
swrl_conjunction([Atom])      --> swrl_literal(Atom).

swrl_literal(class(A,B)) -->
  swrl_class_atom(A), ['('], swrl_subject(B), [')'].
swrl_literal(property(S,P,O)) -->
  swrl_property(P), ['('], swrl_subject(S), [','], swrl_data_object(O), [')'],
  { is_datatype_property(P) }.
swrl_literal(property(S,P,O)) -->
  swrl_property(P), ['('], swrl_subject(S), [','], swrl_object(O), [')'],
  { is_object_property(P) }.
swrl_literal(BuiltinTerm) -->
  { (ground(BuiltinTerm), BuiltinTerm =.. [Predicate|Args]) ; true },
  swrl_builtin(Predicate), ['('], swrl_builtin_args(Args), [')'],
  { BuiltinTerm =.. [Predicate|Args] }.

swrl_class_atom(not(Cls))           --> ['('], ['not'], swrl_class_atom(Cls), [')'].
swrl_class_atom(allOf([X|Xs]))      --> ['('], swrl_class_atom_terminal(X), ['and'], swrl_class_intersection(Xs), [')'].
swrl_class_atom(oneOf([X|Xs]))      --> ['('], swrl_class_atom_terminal(X), ['or'], swrl_class_union(Xs), [')'].
swrl_class_atom(some(P,Cls))        --> ['('], swrl_property(P), ['some'], swrl_class_atom(Cls), [')'].
swrl_class_atom(all(P,Cls))         --> ['('], swrl_property(P), ['all'],  swrl_class_atom(Cls), [')'].
swrl_class_atom(value(P,Value))     --> ['('], swrl_property(P), ['value'], swrl_value(Value), [')'].
swrl_class_atom(exactly(Num,P,Cls)) --> ['('], swrl_property(P), ['exactly'], swrl_number(Num), swrl_class_atom_terminal(Cls), [')'].
swrl_class_atom(max(Num,P,Cls))     --> ['('], swrl_property(P), ['max'],     swrl_number(Num), swrl_class_atom_terminal(Cls), [')'].
swrl_class_atom(min(Num,P,Cls))     --> ['('], swrl_property(P), ['min'],     swrl_number(Num), swrl_class_atom_terminal(Cls), [')'].
swrl_class_atom(Cls)                --> swrl_class_atom_terminal(Cls).
swrl_class_atom_terminal(Cls)       --> [Cls_name], { swrl_match_instance(Cls, Cls_name) }.

swrl_class_intersection([Cls])      --> swrl_class_atom(Cls).
swrl_class_intersection([Cls|Rest]) --> swrl_class_atom(Cls), ['and'], swrl_class_intersection(Rest).
swrl_class_union([Cls])             --> swrl_class_atom(Cls).
swrl_class_union([Cls|Rest])        --> swrl_class_atom(Cls), ['or'], swrl_class_union(Rest).

swrl_subject(S)    --> swrl_var_expr(S).
swrl_subject(S)    --> swrl_individual(S).
swrl_property(P)   --> [P_name], { swrl_match_instance(P, P_name) }.
swrl_object(Var)   --> swrl_var_expr(Var).
swrl_object(Obj)   --> swrl_individual(Obj).
swrl_object(Cls)   --> swrl_class_atom_terminal(Cls).

swrl_individual(I) --> [I_name], { swrl_match_instance(I, I_name) }.
swrl_number(Num)   --> [Num], { number(Num) }.

swrl_value(Val)   --> swrl_individual(Val).
swrl_value(Val)   --> swrl_class_atom_terminal(Val).
swrl_value(Val)   --> swrl_data_object(Val).

swrl_data_object(Var)                                                             --> swrl_var_expr(Var).
swrl_data_object(literal(type('http://www.w3.org/2001/XMLSchema#integer',Val)))   --> [Val], { integer(Val) }.
swrl_data_object(literal(type('http://www.w3.org/2001/XMLSchema#float',Val)))     --> [Val], { float(Val) }.
swrl_data_object(literal(type('http://www.w3.org/2001/XMLSchema#boolean',Val)))   --> [Val].
swrl_data_object(literal(type('http://www.w3.org/2001/XMLSchema#string',Val)))    --> ['"'], [Val], ['"'].
swrl_data_object(literal(Val)) --> [Val], { atom(Val) }.

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
  clause(swrl:swrl_builtin_pl(Term,_,_),_),
  Term =.. [Predicate|_].

swrl_match_instance(Iri,Name) :-
  atom(Iri),
  rdf_split_url(_, Name, Iri).

swrl_match_instance(Iri,Name) :-
  var(Iri), atom(Name),
  rdf_current_prefix(_, Uri),
  rdf_split_url(Uri, Name, Iri),
  % FIXME: do not use rdf_resource
  rdf_resource(Iri).
