
:- module(knowrob_model_Parameter,
    [
      parameter_create/3
    ]).
/** <module> Interface to RDF model of parameters.

*Parameter* is defined as a *Concept* that *classifies* a *Region*; the difference between a *Region* and a *Parameter* is that regions represent sets of observable values, e.g. the height of a given building, while parameters represent constraints or selections on observable values, e.g. 'VeryHigh'. Therefore, parameters can also be used to constrain regions, e.g. *VeryHigh* on a subset of values of the *Region* *Height* applied to buildings, or to add an external selection criterion , such as measurement units, to regions, e.g. *Meter* on a subset of values from the *Region* *Length* applied to the *Region* *Length* applied to roads.

@author Daniel Beßler
@license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/knowrob')).

:- rdf_meta
      parameter_create(r,r,+).

%%
parameter_create(Type,Individual,Graph) :-
  kb_create(Type,Individual,_{graph:Graph}).
