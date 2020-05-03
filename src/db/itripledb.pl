
:- interface(itripledb,
        [ tripledb_init/0,
          tripledb_import/1,             % +Directory
          tripledb_export/1,             % +Directory
          tripledb_whipe/0,
          tripledb_load_rdf(t,+,+),      % +RDF_data, +Data_Scope, +Graph
          tripledb_tell(r,r,r,+,+),      % +Subject, +Property, +Value, +Fact_Scope, +Graph
          tripledb_ask(r,r,r,+,-,?),     % ?Subject, ?Property, ?Value, +Query_Scope, -Fact_Scope, ?Graph
          tripledb_forget(r,r,r,+,?),    % ?Subject, ?Property, ?Value, +Query_Scope, ?Graph
          tripledb_subgraph_of/2,
          %%
          tripledb_cache_get(+,-),
          tripledb_cache_add(+,+,+),
          tripledb_cache_invalidate(+)
        ]).
/** <interface> tripledb interface.

@author Daniel Beßler
@license BSD
*/
