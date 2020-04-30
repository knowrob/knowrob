
:- interface(iobda,
        [ can_access(r),     % ?Property
          access(r,r,r,t,t), % ?Subject, ?Property, ?Value, +Query_Scope, -Fact_Scope
          import/1,          % +Directory
          export/1,          % +Directory
          whipe/0
        ]).
/** <interface> OBDA interface.

@author Daniel Beßler
@license BSD
*/
