
:- module(parsing_test, [
     robcog_test/2,
     robcog_load_map/2,
     robcog_load_episode/4,
     robcog_data_stats/2
]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).

:- use_module(library('robcog-flanagan')).
:- use_module(library('tokenizer')).

:- rdf_db:rdf_register_ns(u_map, 'http://knowrob.org/kb/u_map.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(u_log, 'http://knowrob.org/kb/unreal_log.owl#', [keep(true)]).

:- rdf_meta robcog_load_episode(+,r,+,r).

episode_path('/home/daniel/Downloads/VR-daniel/SemLog').

robcog_load_map(Parser,Map) :-
  episode_path(EpisodePath),
  atomic_list_concat([EpisodePath,'/SemanticMap.owl'], Path),
  owl_parse(Path,Parser),
  rdf_equal(Map,u_map:'SemanticEnvironmentMap_Mg4o'),
  flanaganize_map(Map).

robcog_load_episode(Parser,Map,Name,Episode) :-
  episode_path(EpisodePath),
  atomic_list_concat([EpisodePath,'/Episodes/',Name,'/log.owl'], Path),
  owl_parse(Path,Parser),
  rdf(Episode,rdf:type,knowrob:'UnrealExperiment',Parser),
  flanaganize_episode(Map,Episode).

robcog_test(EpisodeName,Interpretation) :-
  parser_create(Parser),
  robcog_load_map(Parser,Map),
  robcog_load_episode(Parser,Map,EpisodeName,Episode),
  tokenize(Episode, Tokens),
  detect_activity2(Parser,Tokens,Interpretation)
  %, parser_retract(Parser)
  .

robcog_data_stats(TotalNumTokens,TotalDuration) :-
  Parser='*',
  Episodes=[
    a1,a2,a3,a4,a5,a6,a7,a8,a9,a10 %,
    %b1,b2,b3,b4,b5,b6,b7,b8,b9,b10
  ],
  robcog_load_map(Parser,Map),
  findall(TokCount-D, (
    member(Name,Episodes),
    writeln(['stats for',Name]),
    once((
      robcog_load_episode(Parser,Map,Name,Episode),
      interval(Episode,[Begin,End]),
      D is End - Begin,
      tokenize(Episode,Tokens),
      length(Tokens,TokCount)
    )),
    writeln(['-->',TokCount,D])
    ),
    Data),
  findall(Count, member(Count-_,Data), C),
  sumlist(C,TotalNumTokens),
  findall(D, member(_-D,Data), Durations),
  sumlist(Durations,TotalDuration).

%:- begin_tests(parsing_test).

%test('robcog_test(a2)', [nondet]) :-
  %robcog_test(a2,Interpretations).

%:- end_tests(parsing_test).
