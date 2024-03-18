\page prolog Prolog

Prolog-based inference integrates nicely in KnowRob query evaluation.
The data structures used by KnowRob can easily be mapped to Prolog terms and ice versa.

Prolog uses its owl in-memory triple store, and the common way would be to load triples from the KnowRob triple store into Prolog
such that Prolog can reason over them.
However, Prolog can also interact with other data backends in theory, but due to the way how query evaluation works via backtracking,
the overhead of performing evaluation with external data backends can be high.
