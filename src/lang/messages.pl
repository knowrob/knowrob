:- module(mng_messages, []).

prolog:message(lang(_Failed, compilation_failed(Term, _Context))) -->
	[ 'Compilation failed at command `~w`.'-[Term] ].

prolog:message(lang(Failed, expansion_failed(Context))) -->
	[ 'Expansion failed at goal `~w` within `~w`.'-[Failed,Context] ].

prolog:message(lang(Failed, assertion_failed(Body))) -->
	[ 'Assertion of rule `~w` failed for clause `~w`.'-[Failed,Body] ].

prolog:message(db(read_only(Predicate))) -->
	[ 'Predicate `~w` tried to write despite read only access.'-[Predicate] ].
