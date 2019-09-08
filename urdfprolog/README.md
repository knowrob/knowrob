urdfprolog
===

A Prolog-based interface to the Unified Robot Description Format (URDF).
The package first implements a wrapper around the C++ URDF parser library,
and second a mapping into a OWL model of URDF.
The OWL model is defined in an ontology that comes with this package.

Here is an example usecase of urdfprolog where RDF descriptions
of a PR2 robot are generated:

    ros_package_path('urdfprolog', X),
    atom_concat(X, '/urdf/pr2_for_unit_tests.urdf', FileURL),
    kb_create(urdf:'Robot', Robot),
    rdf_urdf_load(Robot, FileURL).
  
  The result is a RDF description of the robot's links and joints.
  However, components are not considered in URDF.
  For component descriptions, please have a look at the SRDL package.
