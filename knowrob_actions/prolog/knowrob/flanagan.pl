
% TODO: this should move to knoworb_common partly?

:- module(flanagan, [
     event_constituents/3,
     event_boundary_constraints/2,
     assert_interval/3,
     assert_limb_motion/4,
     assert_grasping_motion/4,
     assert_releasing_motion/4,
     assert_locomotion/4,
     assert_has_interval/3,
     physical_actions/1,
     physical_actions/2
]).

:- rdf_db:rdf_register_ns(dul, 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).

:- rdf_db:rdf_register_ns(allen, 'http://www.ease.org/ont/allen.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(motions, 'http://www.ease.org/ont/motions.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(actions, 'http://www.ease.org/ont/actions.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(ease, 'http://www.ease.org/ont/ease.owl#', [keep(true)]).

:- rdf_meta event_constituents(r,t,t),
            event_boundary_constraints(r,t),
            assert_motion(+,r,t,+,-).

allen_constraint(S,P,O,Term) :-
  rdf_has_prolog(P,allen:symbol,Sym),
  Term=..[Sym,S,O].

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% 'Time interval'

assert_interval(G, [Begin,End], Interval) :-
   rdf_instance_from_class(dul:'TimeInterval',G,Interval),
   atom_number(BeginAtom,Begin),
   atom_number(EndAtom,End),
   rdf_assert(Interval, allen:hasIntervalBegin,
              literal(type(xsd:float,BeginAtom)), G),
   rdf_assert(Interval, allen:hasIntervalEnd,
              literal(type(xsd:float,EndAtom)), G).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Event

event_constituents(Evt,Constituents,Constraints) :-
  findall(Type-Constraints, (
    rdfs_subclass_of(Evt, Restr),
    % NOTE this assumes particular structure of axioms
    owl_description_recursive(Restr,
        restriction(P, some_values_from(X))),
    rdfs_subproperty_of(P,dul:hasConstituent),
    ( X=intersection_of([Type|Constraints]) ;
    ( Type=class(_), Type=X, Constraints=[] )),
    % HACK better check which ones are subclass of each other and "merge"
    Type=class(Iri),
    \+ rdf_equal(Iri,ease:'PhysicsProcess'),
    \+ rdf_equal(Iri,dul:'Action')),
    TypeConstraints),
  findall(Constituent,
    member(class(Constituent)-_, TypeConstraints),
    Constituents),
  findall(Constraint, (
    member(class(A)-Axioms, TypeConstraints),
    member(restriction(P, some_values_from(B)),Axioms),
    allen_constraint(A,P,B,Constraint)),
    Constraints).

event_boundary_constraints(Evt,Constraints) :-
  findall(Constraint, (
    rdfs_subclass_of(Evt, Restr),
    owl_description(Restr, restriction(P, some_values_from(B))),
    allen_constraint(Evt,P,B,Constraint)),
    Constraints).

% Event 'has interval' some 'Time interval'
assert_has_interval(G, Event, [Begin,End]) :-
  assert_interval(G, [Begin,End], Interval),
  rdf_assert(Event, dul:hasTimeInterval, Interval, G).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Motion

assert_motion(G, MotionType, Participants, Interval, Motion) :-
  rdf_instance_from_class(MotionType,G,Motion),
  forall(member(P,Participants),
         rdf_assert(Motion,dul:hasParticipant,P,G)),
  assert_has_interval(G,Motion, Interval).

assert_limb_motion(G, Limb, Interval, Motion) :-
  assert_motion(G, motions:'LimbMotion', [Limb], Interval, Motion).
assert_grasping_motion(G, Effector, Interval, Motion) :-
  assert_motion(G, motions:'GraspingMotion', [Effector], Interval, Motion).
assert_releasing_motion(G, Effector, Interval, Motion) :-
  assert_motion(G, motions:'ReleasingMotion', [Effector], Interval, Motion).
assert_locomotion(G, Agent, Interval, Motion) :-
  assert_motion(G, motions:'ReleasingMotion', [Agent], Interval, Motion).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% 'Physical action'

% set of all 'Physical action' subclasses without subclass
physical_actions(ActionSet) :-
  physical_actions(ActionSet,user).

physical_actions(ActionSet,RDFGraph) :-
  findall(ActionClass, (
    rdfs_subclass_of(ActionClass, actions:'PhysicalAction'),
    once((
      rdf(ActionClass, rdfs:subClassOf, _, RDFGraph),
      \+ rdf(_, rdfs:subClassOf, ActionClass)
    ))
  ), Actions),
  list_to_set(Actions, ActionSet).
