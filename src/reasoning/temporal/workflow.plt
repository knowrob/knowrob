
:- begin_tests('temporal_workflow').

%:- use_module(library('semweb/owl_parser')).
%:- use_module(library('knowrob/reasoning/task_planning')).
%:- use_module(library('knowrob/model/Workflow'), [ workflow_role_range/3 ]).

%:- owl_parse('package://knowrob/owl/test/pancake.owl').

%:- rdf_db:rdf_register_ns(pancake, 'http://knowrob.org/kb/pancake.owl#', [keep(true)]).

%test('WF_MakingPancakes_0 sequence', [nondet]) :-
  %workflow_sequence(pancake:'WF_MakingPancakes_0', [
    %pancake:'Mixing_0',
    %pancake:'Baking_0'
  %]).

%test('WF_Baking_0 sequence', [nondet]) :-
  %workflow_sequence(pancake:'WF_Baking_0', [
    %pancake:'TurningOn_0',
    %pancake:'Pouring_0',
    %pancake:'FlippingAPancake_0'
  %]).

%test('WF_Mixing_0 roles', [nondet]) :-
  %workflow_role_range(pancake:'WF_Mixing_0', _, pancake:'Egg'),
  %workflow_role_range(pancake:'WF_Mixing_0', _, pancake:'Milk'),
  %workflow_role_range(pancake:'WF_Mixing_0', _, pancake:'WheatFlour'),
  %workflow_role_range(pancake:'WF_Mixing_0', _, pancake:'EggYolk'),
  %workflow_role_range(pancake:'WF_Mixing_0', _, pancake:'Dough').

:- end_tests('temporal_workflow').
