:- module(model_SOMA_IO,
    [ is_computational_agent(r),
      is_digital_object(r),
      is_kino_dynamic_data(r),
      has_kinematics_file(r,?,?)
    ]).
/** <module> ....

@author Daniel Beßler
@license BSD
*/

:- use_module(library('model/RDFS'),
    [ has_type/2 ]).

%% is_computational_agent(?Entity) is semidet.
%
% True iff Entity is an instance of soma:'ComputationalAgent'.
%
% @param Entity An entity IRI.
%
is_computational_agent(Entity) ?+>
	has_type(Entity, soma:'ComputationalAgent').

%% is_digital_object(?Entity) is semidet.
%
% True iff Entity is an instance of soma:'DigitalObject'.
%
% @param Entity An entity IRI.
%
is_digital_object(Entity) ?+>
	has_type(Entity, soma:'DigitalObject').

%% has_kinematics_file(?OBJ,?DOI,?Format) is semidet.
%
is_kino_dynamic_data(IO) ?+>
	has_type(IO, soma:'KinoDynamicData').

%% has_kinematics_file(?OBJ,?DOI,?Format) is semidet.
%
% Associates an object to KinoDynamicData about the object.
%
% @param Obj An entity IRI.
% @param Identifier The DOI of a data file.
% @param Format File format identifier string (i.e. the file extension).
%
has_kinematics_file(Obj,Identifier,Format) +>
	has_type(IO,soma:'KinoDynamicData'),
	has_type(IR,io:'DigitalResource'),
	triple(IO, dul:isAbout, Obj),
	triple(IR, dul:realizes, IO),
	triple(IR, soma:hasPersistentIdentifier, Identifier),
	triple(IR, soma:hasDataFormat, Format).

has_kinematics_file(Obj,Identifier,Format) ?>
	triple(IO, dul:isAbout, Obj),
	has_type(IO,soma:'KinoDynamicData'),
	triple(IR, dul:realizes, IO),
	triple(IR, soma:hasPersistentIdentifier, Identifier),
	triple(IR, soma:hasDataFormat, Format).
