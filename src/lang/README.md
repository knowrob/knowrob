knowrob_lang
=======

The KnowRob language is mainly made of two predicates: `ask/2` and `tell/2`.
Their first argument is a language term (such as `holds/3`),
and their second argument is a contextual parameter used
to handle scoped statements and questions.

### Language terms

The binary operators
`?>` (ask operator) and
`+>` (tell operator) are used to declare language terms, and
how they are hooked into the ask and tell predicates respectively.
Syntactically, these declarations are similar to regular Prolog rules,
which, instead, use the operator `:-`.
When such a declaration appears in a module, the code is 
expanded into additional clauses of the ask and tell predicates.
A regular Prolog rule is also generated, but the body of the rule
is replaced with a call of the ask or tell predicate with the respective 
term as argument.

Each call within the body of such a rule is expanded to a call of
the ask or tell predicate instead where the language term argument
is instantiated to the term called in the original body of the rule.
So if we write, for example, `is_object(OBJ)` on the right-hand side
of an ask rule, it is internally expanded to `ask(is_object(OBJ),Context)`.
In the case of a tell rule, above expression would expand to
`tell(is_object(OBJ),Context)`, which is also meaningful.
In such a case, where ask and tell rules can be declared identical
within the ask and tell operators, one can also use the ask-tell operator
`?+>` which expands into both cases to avoid code redundancy.

KnowRob pre-defines some general language terms that are used to relate entities
in the knowledge base to each other,
or to assign data properties to them.
These are:

| Term | Description |
| --- | --- |
| `triple/3`           | Triple data in the triple store |
| `holds/3`            | A relation between entities, or a data property of an entitiy |
| `is_a/2`             | A taxonomical relation between entities and classes |
| `instance_of/2`      | The type-of relationship between entities and classes |
| `subclass_of/2`      | The subclass-of relationship |
| `subproperty_of/2`   | The subproperty-of relationship |
| `occurs/1`           | An event occurence |
| `is_at/2`            | A spatial relation between entities and locations |
| `transitive/1`       | A transitive relation between entities |

### Scoped questions and statements

Another aspect of these operators is that they hide the contextual
argument from the declaration (above: the `Context` argument of predicates).
This is handy for declaring context-invariant rules.
However, rules may be written that retrieve or update the context hidden
in the operator.
This is possible through a DSL that is designed to interact
with the hidden context argument.

Within tell rules, context is a tuple of
fact scope (also called statement scope), and configuration.
The scope of the statement restricts the context
in which the statement is true, for example,
that some statement is true after some event has happened.
The scope must be instantiated before calling the tell predicate.
Statements in tell rules expand into scoped assertions in the triple store.
Within ask rules, on the other hand, context has an additional 
query scope (also called question scope).
Question scopes are used to restrict the scope
of considered statements to statements with overlapping
scopes to the one provided.
The ask rule instantiates the statement scope to some scope
within which the given statement is true, if any.
In case of the term expands into a conjunctive query,
the statament scope is instantiated to the intersection
of individual statement scopes (if the intersection is not empty).

Special scoping language terms are used to restrict the scope
within a nested term. Restrictions of the scope will allways be passed
through to child terms, hence the scoping is done in the outer term such as
in `since(holds(S,P,O),Time)`. As these terms are usually defined as operators,
one can also write, for example, `holds(S,P,O) since Time`.
Generally, scoping terms are nested terms where another language term is
called with an updated scope.
In the case of ask-rules, scoping predicates will modify the question scope,
while, in the case of tell-rules, the statement scope is modified.

KnowRob pre-defines a time scope, and the following time-scoping terms:

| Term | Description |
| --- | --- |
| `during/2` | The time frame in which a statement holds |
| `since/2`  | The begin time of a statement being true |
| `until/2`  | The end time of a statement being true |
