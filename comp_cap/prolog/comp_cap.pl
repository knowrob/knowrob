/**
@author Steffen Neumann
*/

:- module(comp_cap,
    [  
      comp_capability/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('jpl')).
:- use_module(library('srdl2')).
:- jpl_set_default_jvm_opts(['-Xmx2048M']).

:- owl_parser:owl_parse('../owl/comp_cap.owl', false, false, true).

:- rdf_db:rdf_register_ns(srdl_capabilty, 'http://ias.cs.tum.edu/kb/SRDL_capability.owl',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob,  'http://ias.cs.tum.edu/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(comp_cap, 'http://ias.cs.tum.edu/kb/comp_cap.owl#', [keep(true)]).

%
% descritpiton
%
% @param
% @param 
% 
comp_capability(Inst, Class) :-
  jpl_new('edu.tum.cs.ias.knowrob.comp_cap.CapabilityBase',[], CapabilityBase),
  findall(SubTopic,srdl2:class_properties(Class, comp_cap:dependsOnSubscribedTopic, SubTopic), SubTopics),
  jpl_list_to_array(SubTopics, JPLSubTopics),
  findall(PubTopic,srdl2:class_properties(Class, comp_cap:dependsOnPublishedTopic, PubTopic), PubTopics),
  jpl_list_to_array(PubTopics, JPLPubTopics),
  jpl_call(CapabilityBase, 'comp_capability',[JPLPubTopics, JPLSubTopics], Res),
  jpl_array_to_list(Res,MisTopics),
  length(MisTopics,X),
  X = 0,
  rdf_instance_from_class(Class, Inst).
  
