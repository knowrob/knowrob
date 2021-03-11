mongolog
=======

The main purpose of `mongolog` is to run logic programs
with facts that are stored in a mongo database.
Logic programs are translated into *aggregation pipelines*
and executed server-side.
This brings the advantage of *horizontal scalability*
in case of *sharded data*, and keeps the IO troughput small
as the data is transformed and combined server-side.

The current implementation has some limitations and does not
support the full ISO standard.
One of the limitations is that rules cannot be asserted
into the mongo database, but are baked into the aggregation pipeline.
Another limitation is that *recursion in rules is not allowed*.
The reason is that support for recursion in the mongo DB aggregation framework is
weak (for version <= 4.4).
However, many other aspects are supported by `mongolog`:

- conjunctive and disjunctive queries;
- negation as failure;
- cut operator;
- arithmetic expressions and operators;
- data structures: atoms, numbers, compound terms, lists, variables;
- comparison operators;
- control and meta predicates;
- findall and list predicates;
- typechecking commands;
- database interaction (for facts only);
- unification of compound terms (does not handle implicit instantiation yet).
