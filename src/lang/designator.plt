
:- begin_tests(designator).

test(designator) :-
  fail.

%:- use_module(library('semweb/rdf_db')).
%:- use_module(library('semweb/rdfs')).
%:- use_module(library('semweb/owl_parser')).
%:- use_module(library('knowrob/lang/entity')).

%:- owl_parser:owl_parse('package://knowrob/owl/test/test.owl').

%:- rdf_db:rdf_register_prefix(entity_test, 'http://knowrob.org/kb/entity_test.owl#', [keep(true)]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% OWL entity descriptions

%%% Events

%%% Objects

%test(generate_refrigerator_description) :-
  %entity(entity_test:'Refrigerator_fg45543', X),
  %X = [an, object, [type, ease:refrigerator]].

%test(query_refrigerator, [nondet]) :-
  %entity(Cont, [an, object, [type, ease:refrigerator]]),
  %rdf_equal(Cont, entity_test:'Refrigerator_fg45543').

%test(query_cup, [nondet]) :-
  %entity(Cup, [an, object, [type, ease_obj:'Crockery']]),
  %rdf_equal(Cup, entity_test:'Cup_sfd498th').

%test(query_disposition, [nondet]) :-
  %entity(Obj, [an, object,
    %[ease_obj:hasDisposition, [
      %[type,ease_obj:'Insertion']
    %]]
  %]),
  %rdf_equal(Obj, entity_test:'Refrigerator_fg45543').

%test(query_containerFor, [nondet]) :-
  %entity(Obj, [an, object,
    %[ease_obj:hasDisposition, [
      %[type,ease_obj:'Insertion'],
      %[ease_obj:affordsTrigger, [
        %[classifies, only(ease:'DesignedContainer')]
      %]]
    %]]
  %]),
  %rdf_equal(Obj, entity_test:'Refrigerator_fg45543').

%test(query_cup_by_name, [nondet]) :-
  %entity(Cont, [an, object, [name, entity_test:'Cup_sfd498th']]),
  %rdf_equal(Cont, entity_test:'Cup_sfd498th').

%test(query_cup_by_nameString_prop, [nondet]) :-
  %entity(Cont, [an, object, [label, 'cup_name']]),
  %rdf_equal(Cont, entity_test:'Cup_sfd498th').

%test(query_cup_by_nameString_prop_2, [nondet]) :-
  %entity(Cont, [an, object, [label, X]]),
  %X = 'cup_name',
  %rdf_equal(Cont, entity_test:'Cup_sfd498th').
  
:- end_tests(designator).
