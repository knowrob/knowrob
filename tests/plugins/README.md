
#### *LPN Reasoner* (C++)

`LPNReasoner.cpp` demonstrates how a simple reasoner that defines a **computable
RDF property** can be implemented in C++.
The reasoner is implemented as a class that inherits from the `RDFReasoner` class.
This baseclass should be used in case the reasoner is used for the construction of RDF
knowledge graphs. RDF reasoner is defined as a type of top-down reasoner that
computes the value of a property based on the values of other properties or external data.
This is done on demand given a query whose evaluation depends on the value of the computable
property.

Some general technical notes:
- The reasoner must initially *define* the computable property by calling `defineProperty()`.
- Reasoner have optional features allowing KnowRob some optimizations in case they are available.
  These features can be enabled by calling `enableFeature()`. e.g. evaluation of conjunctive queries
  is an optional feature that must be enabled by the reasoner or else KnowRob will only pass simple queries
  containing a single triple pattern to the reasoner.
- `initializeReasoner()` provides a generic interface for properties, no schema is used at the moment.
  You can add any parameter you want to JSON configuration file and read it in this function.

Some C++-specific technical notes:
- This is an "external" reasoner implementation which can optionally be linked into KnowRob. As such, it is
  compiled as a shared library. KnowRob will load the reasoner at runtime in case it is
  configured to do so.
- The reasoner must call the `REASONER_PLUGIN` macro to define the entry point for the shared library.
  This effectively defines a symbol with a fixed name that KnowRob will look for when loading the reasoner.
- Make sure your C++ reasoner plugin can be found by the linker. This can be done by adding the path to the
  plugin to the `LD_LIBRARY_PATH` environment variable. KnowRob also looks into "share/knowrob/reasoners"
  directory and the user's home directory for reasoner plugins.
