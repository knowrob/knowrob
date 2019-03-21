/*
  Copyright (C) 2019 Daniel Beßler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

:- module(kb_querying,
    [
      kb_querying/4
    ]).
/** <module> The execution of KB querying actions.

@author Daniel Beßler
@license BSD
*/
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/actions')).
:- use_module(library('knowrob/action_execution')).

:- rdf_meta kb_querying(r,t,t,-).

% extend action library
action_execution:action_registry('http://knowrob.org/kb/knowrob.owl#KBQuerying', kb_querying).

%% kb_querying(+Action) is semidet.
%
% Executes a querying action by calling a Prolog predicate.
% The predicate must be loaded before.
% The arguments of the predicate call are represented as
% participants of the action
%
% @param Action IRI of querying action
% @param OutputPairs
% 
kb_querying(Action,InputDict,ActionDict,OutputPairs) :-
  %%%%%%%%%
  %%%%% Find KBPredicate participant.
  %%%%%%%%%
  action_call_or_failure(Action, (
      action_participant_type(Action,KBPredicate,knowrob:'KBPredicate')
    ),
    knowrob:'ACTION_INPUT_MISSING',
    'no KBPredicate participant'
  ),
  %%%%%%%%%
  %%%%% OWL -> Prolog
  %%%%%%%%%
  kb_predicate_variables(KBPredicate,KBVariables),
  % find each KBVariable and its assignment
  findall(KBVariable-Assignment, (
    member(KBVariable,KBVariables),
    once((
      action_filler_for(Filler:InputDict,KBVariable:ActionDict),
      owl_entity(Filler,Assignment));
      % argument is an unbound variable
      Assignment=_
    )
  ), Vars),
  %% NOTE: be careful below. Prolog acts kind of unexpected
  %%        when using findall/bagof with lists that hold variables.
  %%       - never only select variables (below trick: make pairs)
  %%       - always use bagof, findall seems to create new vars
  % Arguments of predicates maybe complex terms, create them from OWL
  kb_predicate_arguments(KBPredicate,Vars,Arguments),
  % Remember set of unbound variables in argument terms
  ( bagof(S-X,(member(S-X,Vars), var(X)),OutputVariables) ;
    OutputVariables=[] ),
  %%%%%%%%%
  %%%%% Call the predicate
  %%%%%%%%%
  kb_predicate_indicator(KBPredicate,Arguments,Functor,Indicator),
  %rdf_has_prolog(KBPredicate,knowrob:hasModulePath,ModulePath),
  action_call_or_failure(Action, (
      current_predicate(Indicator)
    ),
    knowrob:'ACTION_MODEL_ERROR',
    'predicate unknown'
  ), !,
  pairs_values(Arguments,Arguments_),
  %kb_query(Functor, Arguments, Query),
  Query =.. [Functor|Arguments_],
  call(Query),
  %%%%%%%%%
  %%%%% Prolog -> OWL
  %%%%%%%%%
  findall(Role-Assignment_owl, (
    member(KBVariable-OutAssignment,OutputVariables),
    % pl to owl
    create_owl_entity(OutAssignment,Assignment_owl),
    get_dict(Role, ActionDict, KBVariable),
    % assign as participant/region of the action
    action_add_filler(Action,Assignment_owl)
  ), OutputPairs).

% predicate indicator atom '$functor/$arity'
kb_predicate_indicator(KBPredicate,Arguments,Functor,Indicator) :-
  rdf_has_prolog(KBPredicate,knowrob:hasFunctor,Functor),
  length(Arguments, Arity), term_to_atom(Arity, Arity_atom),
  atomic_list_concat([Functor,Arity_atom],'/', Indicator_atom),
  term_to_atom(Indicator,Indicator_atom).

% collect sequence of arguments
kb_predicate_arguments(KBPredicate,VarAssignments,Arguments) :-
  %findall(X,member([_,X],SlotArgs),Arguments),
  rdf_has(KBPredicate,knowrob:firstKBTerm,FirstTerm),!,
  owl_list_to_pl(FirstTerm,Terms),
  bagof(T-Arg, (
    member(T,Terms),
    kb_term_argument(T,VarAssignments,Arg)
  ),Arguments).

% 
kb_term_argument(Term,Vars,Arg) :-
  rdfs_individual_of(Term,knowrob:'KBVariable'),!,
  member(Term-Arg,Vars).
kb_term_argument(Term,_Vars,Arg) :-
  rdfs_individual_of(Term,knowrob:'KBAtomic'),!,
  rdf_has_prolog(Term,dul:hasDataValue,Arg).
kb_term_argument(T_compound,Vars,Arg) :-
  rdf_has_prolog(T_compound,knowrob:hasFunctor,Functor),
  rdf_has(T_compound,knowrob:firstKBTerm,_),!,
  kb_predicate_arguments(T_compound,Vars,T_args),
  Arg =.. [Functor|T_args].

% collect sequence of variables
kb_predicate_variables(KBPredicate,Vars_set) :-
  rdf_has(KBPredicate,knowrob:firstKBTerm,FirstTerm),!,
  owl_list_to_pl(FirstTerm,Terms),
  findall(Var, (
    member(T,Terms),
    kb_term_variable(T,Var)),
    Vars),
  list_to_set(Vars,Vars_set).
kb_predicate_variables(_,[]). % 0-ary clause

% 
kb_term_variable(T_var,T_var) :-
  rdfs_individual_of(T_var,knowrob:'KBVariable'),!.
kb_term_variable(T_compound,Var) :-
  rdf_has(T_compound,knowrob:firstKBTerm,FirstTerm),!,
  owl_list_to_pl(FirstTerm,Terms),
  member(T,Terms),
  kb_term_variable(T,Var).
