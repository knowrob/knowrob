/**
@author Steffen Neumann
*/

:- module(comp_cap,
    [
      comp_move_base/2,   
      comp_move_arm/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('jpl')).
:- jpl_set_default_jvm_opts(['-Xmx2048M']).

:- owl_parser:owl_parse('../owl/comp_cap.owl', false, false, true).

:- rdf_db:rdf_register_ns(srdl_capabilty, 'http://ias.cs.tum.edu/kb/SRDL_capability.owl',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob,  'http://ias.cs.tum.edu/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(comp_cop, 'http://ias.cs.tum.edu/kb/comp_cap.owl#', [keep(true)]).


%
% descritpiton
%
% @param
% @param 
% 
comp_move_base(Inst, 'http://ias.cs.tum.edu/kb/SRDL_capability.owl#move_base') :-
  jpl_new('edu.tum.cs.ias.knowrob.comp_cap.CapabilityBase',[], CapabilityBase),
  jpl_call(CapabilityBase, 'cap_move_base', [], Res),
  jpl_is_true(Res),
  rdfs_instance_from_class('http://ias.cs.tum.edu/kb/SRDL_capability.owl#move_base',Inst).


%
% descritpiton
%
% @param
% @param 
% 
comp_move_arm(Inst, 'http://ias.cs.tum.edu/kb/SRDL_capability.owl#move_arm') :-
  jpl_new('edu.tum.cs.ias.knowrob.comp_cap.CapabilityBase',[], CapabilityBase),
  jpl_call(CapabilityBase, 'cap_move_arm', [], Res),
  jpl_is_true(Res),
  rdfs_instance_from_class('http://ias.cs.tum.edu/kb/SRDL_capability.owl#move_arm',Inst).