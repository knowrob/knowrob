:- interface(itripledb,
        [ tripledb_init/0,
          tripledb_import/1,             % +Directory
          tripledb_export/1,             % +Directory
          tripledb_drop/0,
          tripledb_tell(r,r,t,+,+),      % +Subject, +Property, +Value, +Fact_Scope, +Options
          tripledb_bulk_tell(t,+,+),     % +Triples, +Fact_Scope, +Options
          tripledb_ask(r,r,t,+,+,-),     % ?Subject, ?Property, ?Value, +Query_Scope, -Fact_Scope, +Options
          tripledb_forget(r,r,r,+,+),    % ?Subject, ?Property, ?Value, +Query_Scope, +Options
          %%
          tripledb_cache_get(+,+,-),
          tripledb_cache_add(+,+,+),
          tripledb_cache_invalidate(+)
        ]).
/** <interface> tripledb interface.

@author Daniel Beßler
@license BSD
*/
