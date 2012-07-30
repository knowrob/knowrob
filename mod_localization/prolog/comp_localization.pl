:- module(comp_localization,
    [
      clocalization/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).


:- rdf_db:rdf_register_ns(knowrob,  'http://ias.cs.tum.edu/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(localization, 'http://ias.cs.tum.edu/kb/localization.owl#', [keep(true)]).




% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% As part of the KnowRob tutorial, you are supposed to implement the
% following two predicates. You can use the predicates below for
% creating the object and perception instances.
%

clocalization(Robot, Loc) :-
    
    % create ROS client object
    jpl_new('edu.tum.cs.ias.knowrob.mod_localization.ROSClient', ['my_localization_client'], Client),

    % call the method for retrieving objects from the tabletop_object_detector
    jpl_call(Client, 'getLocationByPooling', [], Localization_Array),

    % convert the result into a prolog list over which we can iterate
    jpl_array_to_list(Localization_Array, LocList),

    [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33] = LocList,

    atomic_list_concat(['rotMat3D_',M00,'_',M01,'_',M02,'_',M03,'_',M10,'_',M11,'_',M12,'_',M13,'_',M20,'_',M21,'_',M22,'_',M23,'_',M30,'_',M31,'_',M32,'_',M33], LocIdentifier),

    atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', LocIdentifier, Loc),
    rdf_assert(Loc, rdf:type, knowrob:'RotationMatrix3D').

    % set_perception_pose([M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]).

    % rdf_has(Robot, rdf:type, knowrob:'PR2'),
    % atom(coord2),
    
    % term_to_atom(X, _X),
    % rdf_assert(_X, rdf:type, xsd:'float'),
    % rdf_assert(coord2, knowrob:xCoord, _X),

    % term_to_atom(Y, _Y),
    % rdf_assert(_Y, rdf:type, xsd:'float'),
    % rdf_assert(coord2, knowrob:yCoord, _Y),

    % term_to_atom(Z, _Z),
    % rdf_assert(_Z, rdf:type, xsd:'float'),
    % rdf_assert(coord2, knowrob:zCoord, _Z),

    % rdf_assert(coord2, rdf:type, knowrob:'point3D'),
    % rdf_has(Coord, knowrob:xCoord, _X),
    % rdf_has(Coord, knowrob:yCoord, _Y),
    % rdf_has(Coord, knowrob:zCoord, _Z),
    % rdf_has(Coord, rdf:type, knowrob:'point3D').
    % atom_to_term(coord2, Coord, []).


