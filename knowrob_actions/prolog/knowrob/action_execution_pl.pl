
:- module(knowrob_action_execution_pl,
    [
      kb_query/4
    ]).
/** <module> The execution of KB querying actions.

@author Daniel Beßler
*/
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/knowrob')).
:- use_module(library('knowrob/action_execution')).

:- use_module(library('knowrob/model/Event'), [ event_participant/3 ]).
:- use_module(library('knowrob/model/Action'), [ action_add_filler/2, action_filler_binding/2 ]).
:- use_module(library('knowrob/model/Situation'), [ situation_add_assignment/3 ]).

:- rdf_meta kb_querying(r,t,t,-).

% extend action library
knowrob_action_execution:action_registry(
  'http://knowrob.org/kb/knowrob.owl#KBQuerying',
  knowrob_action_execution_pl:kb_query).

% FIXME: in case of non deterministic predicate, the output assignments are all mixed
%        in the situation and cannot be distiniguished.
%        Might need to use sub-situations.

%% kb_query(PlanExecution,BindingDict,InputDict,OutputPairs) is semidet.
%
% Executes a querying action by calling a Prolog predicate.
% The predicate must be loaded before.
% The arguments of the predicate call are represented as
% participants of the action
%
% @param Action IRI of querying action
% @param OutputPairs
% 
kb_query(PlanExecution,BindingDict,InputDict,OutputPairs) :-
  kb_triple(PlanExecution,dul:includesAction,Action),
  %%%%%%%%%
  %%%%% Find KBPredicate participant.
  %%%%%%%%%
  ( event_participant(Action,KBPredicate,knowrob:'ComputationalPredicate') ;
    throw(action_failure(knowrob:'MissingArgument'))
  ),
  %%%%%%%%%
  %%%%% OWL -> Prolog
  %%%%%%%%%
  kb_predicate_variables(KBPredicate,KBVariables),
  % find each KBVariable and its assignment
  findall(KBVariable-Assignment, (
    member(KBVariable,KBVariables),
    once(
      ( action_filler_binding(Filler:InputDict,KBVariable:BindingDict),
        situation_add_assignment(PlanExecution,KBVariable,Filler),
      ( kb_type_of(Filler,dul:'Parameter') ->
        kb_rdf_pl(dul:hasParameter,Filler,Assignment) ;
        kb_rdf_pl(dul:hasParticipant,Filler,Assignment)
      ))
      ; Assignment=_
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
  ( current_predicate(Indicator) ;
    throw(action_failure(knowrob:'UndefinedReference'))
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
    owl_create_pl_entity(OutAssignment,Assignment_owl),
    % assign as participant/region of the action
    action_add_filler(Action,Assignment_owl),
    situation_add_assignment(PlanExecution,KBVariable,Assignment_owl),
    get_dict(Role, BindingDict, KBVariable)
  ), OutputPairs).

% predicate indicator atom '$functor/$arity'
kb_predicate_indicator(KBPredicate,Arguments,Functor,Indicator) :-
  kb_triple(KBPredicate,knowrob:hasFunctor,Functor),
  length(Arguments, Arity), term_to_atom(Arity, Arity_atom),
  atomic_list_concat([Functor,Arity_atom],'/', Indicator_atom),
  term_to_atom(Indicator,Indicator_atom).

% collect sequence of arguments
kb_predicate_arguments(KBPredicate,VarAssignments,Arguments) :-
  %findall(X,member([_,X],SlotArgs),Arguments),
  kb_triple(KBPredicate,knowrob:hasFirstProcedureArgument,FirstTerm),!,
  dul_sequence(FirstTerm,Terms),
  bagof(T-Arg, (
    member(T,Terms),
    kb_term_argument(T,VarAssignments,Arg)
  ),Arguments).

% 
kb_term_argument(Term,Vars,Arg) :-
  kb_type_of(Term,knowrob:'VariableArgument'),!,
  member(Term-Arg,Vars).
kb_term_argument(Term,_Vars,Arg) :-
  kb_type_of(Term,knowrob:'AtomicArgument'),!,
  kb_triple(Term,dul:hasDataValue,Arg).
kb_term_argument(T_compound,Vars,Arg) :-
  kb_triple(T_compound,knowrob:hasFunctor,Functor),
  kb_triple(T_compound,knowrob:hasFirstProcedureArgument,_),!,
  kb_predicate_arguments(T_compound,Vars,T_args),
  Arg =.. [Functor|T_args].

% collect sequence of variables
kb_predicate_variables(KBPredicate,Vars_set) :-
  kb_triple(KBPredicate,knowrob:hasFirstProcedureArgument,FirstTerm),!,
  dul_sequence(FirstTerm,Terms),
  findall(Var, (
    member(T,Terms),
    kb_term_variable(T,Var)),
    Vars),
  list_to_set(Vars,Vars_set).
kb_predicate_variables(_,[]). % 0-ary clause

% 
kb_term_variable(T_var,T_var) :-
  kb_type_of(T_var,knowrob:'VariableArgument'),!.
kb_term_variable(T_compound,Var) :-
  kb_triple(T_compound,knowrob:hasFirstProcedureArgument,FirstTerm),!,
  dul_sequence(FirstTerm,Terms),
  member(T,Terms),
  kb_term_variable(T,Var).

dul_sequence(X,[X|Xs]) :-
  rdf_has(Y,dul:follows,X),!,
  dul_sequence(Y,Xs).
dul_sequence(X,[X]).

%% 
owl_create_pl_entity([
    RefFrame, _TargetFrame,
    [X,Y,Z],
    [QX,QY,QZ,QW]],Transform) :- !,
  kb_create(knowrob:'Pose',Transform),
  ( object_frame_name(Parent,RefFrame) ->
    rdf_assert(Transform, knowrob:'relativeTo', Parent) ; true ),
  kb_assert(Transform, knowrob:translation, [X,Y,Z]),
  kb_assert(Transform, knowrob:quaternion,  [QX,QY,QZ,QW]).
owl_create_pl_entity(List,Arg_owl) :-
  is_list(List),!,
  % TODO: need to also support constructing e.g. pose regions here.
  kb_create(dul:'Collection',Arg_owl),
  forall(member(X,List),(
    owl_create_pl_entity(X,X_owl),
    rdf_assert(Arg_owl,dul:hasMember,X_owl)
  )).
owl_create_pl_entity(FormalRelation,ReifiedRelation) :-
  atom(FormalRelation),
  rdfs_individual_of(FormalRelation,rdf:'Property'),!,
  kb_reification(FormalRelation,ReifiedRelation).
owl_create_pl_entity(Iri,ReifiedRelation) :-
  atom(Iri),
  rdf_equal(Iri,rdf:type),!,
  kb_reification(Iri,ReifiedRelation).
owl_create_pl_entity(FormalClass,ReifiedClass) :-
  atom(FormalClass),
  rdfs_individual_of(FormalClass,owl:'Class'),!,
  kb_reification(FormalClass,ReifiedClass).
owl_create_pl_entity(RDFClass,ReifiedClass) :-
  atom(RDFClass),
  rdfs_individual_of(RDFClass,rdfs:'Class'),!,
  kb_reification(RDFClass,ReifiedClass).
owl_create_pl_entity(Iri,Iri) :-
  atom(Iri),
  rdfs_individual_of(Iri,owl:'NamedIndividual'),!,
  rdf_has(Iri,_,_),!.
owl_create_pl_entity(Ontology,_) :-
  atom(Ontology),
  rdfs_individual_of(Ontology,owl:'Ontology'),!,
  fail.
owl_create_pl_entity(List,_) :-
  atom(List),
  rdfs_individual_of(List,rdf:'List'),!,
  fail.
owl_create_pl_entity(literal(type(DataType,Atom)),Arg_owl) :-
  kb_create(dul:'Region',Arg_owl),
  rdf_assert(Arg_owl,dul:hasRegionDataValue,literal(type(DataType,Atom))),!.
owl_create_pl_entity(Atom,Arg_owl) :-
  atom(Atom),
  kb_create(dul:'Region',Arg_owl),
  kb_assert(Arg_owl,dul:hasRegionDataValue,Atom),!.
