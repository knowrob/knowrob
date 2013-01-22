%%
%% Copyright (C) 2011 by Stefan Profanter
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%

:- register_ros_package(knowrob_common).

:- register_ros_package(knowrob_cad_models).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parse OWL files, register name spaces

:- owl_parser:owl_parse('@LOCAL_PACKAGE_PATH@/owl/knowrob_cad_models.owl', false, false, true).
:- rdf_db:rdf_register_ns(cad, 'http://ias.cs.tum.edu/kb/knowrob_cad_models.owl#', [keep(true)]).

