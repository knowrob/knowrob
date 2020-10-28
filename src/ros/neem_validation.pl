:- module(neem_logs,
    [ 
      get_actions_without_timeinterval(r),
      get_actions_with_participants_without_role(r)
    ]).

:- use_module(library('db/scope')).
:- use_module(library('db/tripledb'),
    [ tripledb_load/2
    ]).
:- use_module(library('model/DUL/Event')).
:- use_module(library('model/DUL/Object')).
:- use_module(library('model/DUL/Region')).

get_actions_without_timeinterval(ActionWithoutInterval) :-
  findall(Event, is_action(Event), AllActions),
  findall(Action, 
    (has_time_interval(Action, TimeInterval)), 
    ValidActions),
  subtract(AllActions, ValidActions, ActionWithoutInterval),
  length(ActionWithoutInterval, Listlength),
  ( Listlength > 0 -> writeln('The following actions have no time interval'),
  forall(member(Action, ActionWithoutInterval), print_message(warning, Action))); 
  true.

get_actions_with_participants_without_role(ParticipantsWithoutRole) :-
  findall(Event, is_action(Event), AllActions),
  findall(Action, 
    (has_participant(Action, Participant), 
    has_role(Participant, ParticipantRole)), 
    ValidActions),
  subtract(AllActions, ValidActions, ParticipantsWithoutRole),
  length(ParticipantsWithoutRole, Listlength),
  ( Listlength > 0 -> writeln('The following actions have participants with no role. Please assert the corresponding participant role'),
  forall(member(Action, ParticipantsWithoutRole), print_message(warning, Action))); 
  true.

%%% Check if the object frame can be mapped to /map

load_logs(Folder) :-
  get_path(Path, Folder),
  remember(Path),
  tf_mng_remember(Path).


     /*******************************
     *          UNIT TESTS          *
     *******************************/


:- begin_tests('neem_logs').
  
test('load the logs') :-
  load_logs("neems"). % neem is the name of the folder with the logs

test('find all actions without time interval') :-
  get_actions_without_timeinterval(ActionsWithoutTimeInterval).

:- end_tests('neem_logs').