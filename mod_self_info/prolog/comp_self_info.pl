:- module(comp_self_info,
    [
      clocalization/2,
      cbattery_state/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).


:- rdf_db:rdf_register_ns(knowrob,  'http://ias.cs.tum.edu/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(self_info, 'http://ias.cs.tum.edu/kb/self_info.owl#', [keep(true)]).




% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% As part of the KnowRob tutorial, you are supposed to implement the
% following two predicates. You can use the predicates below for
% creating the object and perception instances.
%

clocalization(Robot, Loc) :-
    
    % create ROS client object
    jpl_new('edu.tum.cs.ias.knowrob.mod_self_info.ROSClient_localization', ['my_localization_client'], Client),

    term_to_atom(Robot, pr2),

    jpl_call(Client, 'getLocationByPooling', [], Localization_Array),

    jpl_array_to_list(Localization_Array, LocList),

    ((owl_has(A, rdf:type, knowrob:'PR2'), !) -> (

         rdf_has(pr2, rdf:type, knowrob:'PR2'), !

    ) ; (
    	rdf_assert(pr2, rdf:type, knowrob:'PR2')
    )),

    [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33] = LocList,

    atomic_list_concat(['rotMat3D_',M00,'_',M01,'_',M02,'_',M03,'_',M10,'_',M11,'_',M12,'_',M13,'_',M20,'_',M21,'_',M22,'_',M23,'_',M30,'_',M31,'_',M32,'_',M33], LocIdentifier),

    atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', LocIdentifier, Loc),
    rdf_assert(Loc, rdf:type, knowrob:'RotationMatrix3D'),

    ((rdf_has(pr2, self_info:latestLocalizationInstance, Prev)) -> (

    	rdf_update(pr2, self_info:latestLocalizationInstance, Prev, Loc),
    	rdf_assert(Loc, self_info:previousLocalizationInstance, Prev)

    ) ; (
    rdf_assert(pr2, self_info:latestLocalizationInstance, Loc)
    )).
    

cbattery_state(Robot, Power) :-
    
    % create ROS client object
    jpl_new('edu.tum.cs.ias.knowrob.mod_self_info.ROSClient_battery_state', ['my_battery_client'], Client),

    jpl_call(Client, 'getPowerByPooling', [], Returned_Power),

    term_to_atom(Returned_Power, Y),
    rdf_assert(Y, rdf:type, xsd:'float'),

    % rdf_has(Robot, rdf:type, knowrob:'PR2'),
    rdf_has(Power, rdf:type, xsd:'float'),
 
    atom_to_term(Power, R, []),
    
    A is round(R),
    B is round(Returned_Power),
    
    A = B.


    % rdf_has(Robot, rdf:type, knowrob:'PR2'),
    % X is 0,
    % term_to_atom(X, Y),

    % rdf_assert(Y, rdf:type, xsd:'float'),

    % rdf_has(Power, rdf:type, xsd:'float'),

    % create ROS client object
    % jpl_new('edu.tum.cs.ias.knowrob.mod_battery_state.ROSClient', ['my_battery_client'], Client),

    % call the method for retrieving objects from the tabletop_object_detector
    % jpl_call(Client, 'getPowerByPooling', [], Returned_Power),

    % convert the result into a list of matches over which we can iterate
    % jpl_get(Objects, 'models', Models),

    % term_to_atom(P, Returned_Power),
     
    % atom_to_term(Power, R, []),
    
    % A is round(R),
    
    % B is round(Returned_Power),
    
    % A = B.




