:- module(mng_messages, []).

prolog:message(mongo(compilation_failed(Failed, _AllTerms))) -->
	[ 'Compilation failed at command `~w`.'-[Failed] ].

prolog:message(mongo(compilation_failed(Failed, Context))) -->
	[ 'Expansion failed at goal `~w` within `~w`.'-[Failed,Context] ].

prolog:message(mongo(assertion_failed(Functor, Body))) -->
	[ 'Assertion of rule `~w` failed for clause `~w`.'-[Functor,Body] ].
