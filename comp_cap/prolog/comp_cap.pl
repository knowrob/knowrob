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
:- owl_parser:owl_parse('../owl/comp_cap_topics.owl', false, false, true).
:- owl_parser:owl_parse('../owl/comp_cap_roslaunch.owl', false, false, true).

:- rdf_db:rdf_register_ns(srdl2-cap, 'http://ias.cs.tum.edu/kb/srdl2-cap.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob,  'http://ias.cs.tum.edu/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(comp_cap, 'http://ias.cs.tum.edu/kb/comp_cap.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(comp_cap_topics, 'http://ias.cs.tum.edu/kb/comp_cap_topics.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(comp_cap_roslaunch, 'http://ias.cs.tum.edu/kb/comp_cap_roslaunch.owl#', [keep(true)]).


%
% test the requirements for a given srdl2 capability
% 
% @param
% @param 
%
comp_capability(Inst, Class) :-
  jpl_new('edu.tum.cs.ias.knowrob.comp_cap.CapabilityBase',[], CapabilityBase),
  findall(SubTopic,srdl2:class_properties(Class, comp_cap:dependsOnSubscribedMessageType, SubTopic), SubTopics),
  length(SubTopics,STL),
  STL > 0,
  jpl_list_to_array(SubTopics, JPLSubTopics),
  findall(PubTopic,srdl2:class_properties(Class, comp_cap:dependsOnPublishedMessageType, PubTopic), PubTopics),
  length(PubTopics,PTL),
  PTL > 0,
  jpl_list_to_array(PubTopics, JPLPubTopics),
  findall(Service,srdl2:class_properties(Class, comp_cap:dependsOnService, Service), Services),
  length(Services,SL),
  SL > 0,
  jpl_list_to_array(Services, JPLServices),
  jpl_call(CapabilityBase, 'comp_cap_all',[JPLPubTopics, JPLSubTopics, JPLServices], Res),
  jpl_array_to_list(Res,Namespace),
  length(Namespace,NL),
  X is (STL + PTL + SL),
  NL >= X,
  rdf_instance_from_class(Class, Inst).

comp_capability(Inst, Class) :-
  jpl_new('edu.tum.cs.ias.knowrob.comp_cap.CapabilityBase',[], CapabilityBase),
  findall(SubTopic,srdl2:class_properties(Class, comp_cap:dependsOnSubscribedMessageType, SubTopic), SubTopics),
  length(SubTopics,STL),
  STL > 0,
  jpl_list_to_array(SubTopics, JPLSubTopics),
  findall(PubTopic,srdl2:class_properties(Class, comp_cap:dependsOnPublishedMessageType, PubTopic), PubTopics),
  length(PubTopics,PTL),
  PTL > 0,
  jpl_list_to_array(PubTopics, JPLPubTopics),
  jpl_call(CapabilityBase, 'comp_cap_pub_sub',[JPLPubTopics, JPLSubTopics], Res),
  jpl_array_to_list(Res,Namespace),
  length(Namespace,NL),
  X is STL + PTL,
  NL >= X,
  rdf_instance_from_class(Class, Inst).

comp_capability(Inst, Class) :-
  jpl_new('edu.tum.cs.ias.knowrob.comp_cap.CapabilityBase',[], CapabilityBase),
  findall(SubTopic,srdl2:class_properties(Class, comp_cap:dependsOnSubscribedMessageType, SubTopic), SubTopics),
  length(SubTopics,STL),
  STL > 0,
  jpl_list_to_array(SubTopics, JPLSubTopics),
  findall(Service,srdl2:class_properties(Class, comp_cap:dependsOnService, Service), Services),
  length(Services,SL),
  SL > 0,
  jpl_list_to_array(Services, JPLServices),
  jpl_call(CapabilityBase, 'comp_cap_sub_srvs',[JPLSubTopics ,JPLServices], Res),
  jpl_array_to_list(Res,Namespace),
  length(Namespace,NL),
  X is STL + SL,
  NL >= X,
  rdf_instance_from_class(Class, Inst).

comp_capability(Inst, Class) :-
  jpl_new('edu.tum.cs.ias.knowrob.comp_cap.CapabilityBase',[], CapabilityBase),
  findall(Service,srdl2:class_properties(Class, comp_cap:dependsOnService, Service), Services),
  length(Services,SL),
  SL > 0,
  jpl_list_to_array(Services, JPLServices),
  jpl_call(CapabilityBase, 'comp_cap_srvs',[JPLServices], Res),
  jpl_array_to_list(Res,Namespace),
  length(Namespace,NL),
  NL >= SL,
  rdf_instance_from_class(Class, Inst).

comp_capability(Inst, Class) :-
  jpl_new('edu.tum.cs.ias.knowrob.comp_cap.CapabilityBase',[], CapabilityBase),
  findall(PubTopic,srdl2:class_properties(Class, comp_cap:dependsOnPublishedMessageType, PubTopic), PubTopics),
  length(PubTopics,PTL),
  PTL > 0,
  jpl_list_to_array(PubTopics, JPLPubTopics),
  jpl_call(CapabilityBase, 'comp_cap_pub',[JPLPubTopics], Res),
  jpl_array_to_list(Res,Namespace),
  length(Namespace,NL),
  NL >= PTL,
  rdf_instance_from_class(Class, Inst).

comp_capability(Inst, Class) :-
  jpl_new('edu.tum.cs.ias.knowrob.comp_cap.CapabilityBase',[], CapabilityBase),
  findall(Package,srdl2:class_properties(Class, comp_cap:rospackage, Package), Packages),
  length(Packages,PL),
  PL > 0,
  jpl_list_to_array(Packages, JPLPackages),
  findall(File,srdl2:class_properties(Class, comp_cap:launchFile, File), Files),
  length(Files,FL),
  FL > 0,
  jpl_list_to_array(Files, JPLFiles),
  jpl_call(CapabilityBase, 'roslaunch',[JPLPackages, JPLFiles], Res),
  rdf_instance_from_class(Class, Inst).
