:- module(mng_messages, []).

% a failure during term expansion. Usually caused
% by referring to a predicate that is not known (yet).
prolog:message(lang(expansion_failed(Failed,Mode),_Context)) -->
	[ '~w-query expansion failed at `~w`.'-[Mode,Failed] ].

prolog:message(lang(_Failed, compilation_failed(Term, _Context))) -->
	[ 'Query compilation failed at command `~w`.'-[Term] ].

prolog:message(lang(Failed, assertion_failed(Body))) -->
	[ 'Query assertion of rule `~w` failed for clause `~w`.'-[Failed,Body] ].

% OWL file has been loaded before
prolog:message(db(ontology_detected(Ontology,Version))) -->
	[ 'detected "~w" ontology version ~w.'-[Ontology,Version] ].

% OWL file has been loaded
prolog:message(db(ontology_loaded(Ontology,Version))) -->
	[ 'loaded "~w" ontology version ~w.'-[Ontology,Version] ].

prolog:message(db(read_only(Predicate))) -->
	[ 'Predicate `~w` tried to write despite read only access.'-[Predicate] ].
