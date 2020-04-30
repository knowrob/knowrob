
:- interface(itripledb,
        [ tripledb_init/0,
          tripledb_import/1,             % +Directory
          tripledb_export/1,             % +Directory
          tripledb_whipe/0,
          tripledb_load_rdf(t,t),        % +RDF_data, +Data_Scope
          tripledb_tell(r,r,r,t),        % +Subject, +Property, +Value, +Fact_Scope
          tripledb_ask(r,r,r,t,t),       % ?Subject, ?Property, ?Value, +Query_Scope, -Fact_Scope
          tripledb_forget(r,r,r,t)       % ?Subject, ?Property, ?Value, +Query_Scope
          tripledb_type_of(r,r),         % ?Entity, ?Type
          tripledb_subclass_of(r,r),     % ?Subclass, ?Class
          tripledb_subproperty_of(r,r)   % ?Subproperty, ?Property
        ]).
/** <interface> tripledb interface.

@author Daniel Beßler
@license BSD
*/
