:- module(control_commands, []).

:- use_module(library('lang/compiler')).

%% false
% Same as fail, but the name has a more declarative connotation.
%
false ?+> fail.

%% not(:Goal)
% True if Goal cannot be proven. Retained for compatibility only. New code should use \+/1.
%
not(Exp) ?> \+ Exp.

%% :Condition -> :Action
% If-then and If-Then-Else.
% The ->/2 construct commits to the choices made at its left-hand side,
% destroying choice points created inside the clause (by ;/2),
% or by goals called by this clause.
%
% FIXME: semantics is different when embedded in ;, because the other clause
%            must be prunded by cut here
%
'->'(If,Then) ?> call(If), !, call(Then).
% TODO: support *-> operator, seems difficult
%'*->'(If,Then) ?> call(If), call(Then).

%% query commands
% NOTE: repeat/0 cannot be translated.
%
:- query_compiler:add_command(true,  [ask,tell]).
:- query_compiler:add_command(fail,  [ask,tell]).
:- query_compiler:add_command('!',   [ask,tell]).
:- query_compiler:add_command('+\\', [ask,tell]).

%% query compilation
query_compiler:step_compile(true,  _, []).
query_compiler:step_compile(fail,  _, [['$match', ['$expr', bool(false)]]]).
query_compiler:step_compile(\+(Goal), Context, Pipeline) :-
	/*	\+ :Goal
	    True if‘Goal' cannot be proven (mnemonic: + refers to provable and the backslash
	    (\) is normally used to indicate negation in Prolog).
	
	    Many Prolog implementations (including SWI-Prolog) provide not/1.
	    The not/1 alternative is deprecated due to its strong link to logical negation. 
	    
	    --> lookup, then match next array length=0
	 */
	fail.
query_compiler:step_compile('!', Context, Pipeline) :-
	/*	!
		Cut. Discard all choice points created since entering the predicate in which
		the cut appears. In other words, commit to the clause in which the cut appears
		and discard choice points that have been created by goals to the left of the cut
		in the current clause. Meta calling is opaque to the cut. This implies that cuts that
		appear in a term that is subject to meta-calling (call/1) only affect choice points
		created by the meta-called term. 
	% TODO: cut operator
	%     -- 1. within a clause `cut` wraps terms before
	%             in a lookup with $limit
	%     -- 2. disjunctions keep track of cut in disjuncts
	%             and does not proceed if cut+matching docs in previous step
	*/
	fail.

