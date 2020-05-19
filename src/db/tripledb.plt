:- begin_tests('db/tripledb').

:- use_module(library('db/tripledb'),
    [ tripledb_load/2,
      tripledb_ask/3
    ]).

test('tripledb_load(URL)') :-
  tripledb:tripledb_load('https://protege.stanford.edu/ontologies/pizza/pizza.owl',
    [ graph(common),
      namespace(pizza,'https://protege.stanford.edu/ontologies/pizza/#')
    ]).

:- rdf_db:rdf_register_ns(code_pizza, 'http://www.co-ode.org/ontologies/pizza/pizza.owl#',  [keep(true)]).

test('tripledb_ask(S,P,O)') :-
  tripledb:tripledb_ask(code_pizza:'CheeseyVegetableTopping',
	rdfs:subClassOf,
	code_pizza:'VegetableTopping'
).


:- end_tests('db/tripledb').