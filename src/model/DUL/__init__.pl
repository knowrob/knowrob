
% register ROS packages to resolve IRI prefixes to local paths
:- ros_package_iri(dul,
        'http://www.ontologydesignpatterns.org/ont/dul').
% load RDF data
:- tripledb_load('http://www.ontologydesignpatterns.org/ont/dul/DUL.owl',
    [ graph(tbox),
      namespace(dul)
    ]).

% load modules into user
:- use_module('Object').
:- use_module('Event').
:- use_module('Region').
:- use_module('Situation').
