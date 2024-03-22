\page backends Data Backends

Data backends / storages are used to store *extensional data* representing statements that are true.
KnowRob does not have a default backend, but rather requires the user to configure which backend to use.
The main motivation for this is that reasoner might require specific types of backends, and thus multiple
storages must be supported and potentially synchronized with each other.

Additional backend types can be implemented via plugins that implement a common interface which is
either the `DataBackend` or `QueryableBackend` class.
The `DataBackend` class provides basic functionality for storing and retrieving data, while the `QueryableBackend`
class provides additional functionality for querying the stored data.
Currently, KnowRob supports data backend implementations in the C++ and Python languages.

The following queryable backends are available in KnowRob:

- \subpage redland_backend
- \subpage mongodb_backend
- \subpage PrologBackend
