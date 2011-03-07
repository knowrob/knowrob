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

:- rdf_db:rdf_register_ns(srdl_capabilty, 'http://ias.cs.tum.edu/kb/SRDL_capability.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob,  'http://ias.cs.tum.edu/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(comp_cop, 'http://ias.cs.tum.edu/kb/comp_cap.owl#', [keep(true)]).


%
% descritpiton
%
% @param
% @param 
% 
comp_move_base(Inst, 'http://ias.cs.tum.edu/kb/SRDL_capability.owl#move_base') :-
  write('started comp_move_base'),
  jpl_new('edu.tum.cs.ias.knowrob.comp_cap.CapabilityBase',[], cap_base),
  write('created CapabilityBase'),
  jpl_call(cap_base, 'cap_move_base', ['1'], Res),
  write('called CapabilityBase.comp_move_base'),
  jpl_is_true(Res),
  write('checked Result'),
  rdfs_instance_from_class(srdl_capabilty:move_base,Inst).


%
% descritpiton
%
% @param
% @param 
% 
comp_move_arm(Inst, 'http://ias.cs.tum.edu/kb/SRDL_capability.owl#move_arm') :-
  write('comp_move_arm'),
  jpl_new(class([java,lang],['CapabilityBase']),[''], cap_base),
  jpl_call(cap_base, 'cap_move_arm', ['0'], Res),
  jpl_is_true(Res),
  rdfs_instance_from_class(srdl_capabilty:move_arm,Inst).