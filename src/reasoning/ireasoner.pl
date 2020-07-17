
:- interface(ireasoner,
        [ can_answer/1,  % +Query
          infer/4        % +Query, -Fact, +Query_Scope, +Fact_Scope
        ]).
/** <interface> ....

@author Daniel Beßler
@license BSD
*/
