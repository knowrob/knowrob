:- module(lang_holds,
    [ holds(t),    % ?Query
      holds(r,r,t) % ?Subject, ?Predicate, ?Object
    ]).
/** <module> The *holds* predicate.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('model/XSD'),
    [ xsd_data_basetype/2 ]).
:- use_module(library('model/RDFS'),
    [ has_range/2 ]).
:- use_module(library('model/OWL'),
    [ is_object_property/1,
      is_functional_property/1,
      is_data_property/1
    ]).
:- use_module(library('model/QUDT'),
    [ qudt_unit/4,
      qudt_conv/4
    ]).

:- use_module(library('db/obda'), [ access/2 ]).

%% holds(+Query) is nondet.
%
% Same as holds/3 with arguments wrapped into a single
% term `Property(Subject,Value)`.
%
% @param Query the query term.
%
holds(Query) ?>
  { Query =.. [P,S,O] },
  holds(S,P,O).

%% holds(?Subject, ?Property, ?Value) is nondet.
%
% Query values of a property on some subject.
% Property may also be a list interpreted as a chain of properties,
% thus yielding all values connected to the subject via the chain.
% In case of datatype properties, the value may be a term `Operator(Value)`
% where Operator is a comparison operator (e.g. "<"), meaning that only triples
% are yielded where the comparison between actual value and Value yields true.
%
% NOTE: holds can only be used with OWL ObjectProperties
%       and DatatypeProperties.
%
% @param Subject The subject of a triple.
% @param Property The predicate of a triple.
% @param Value The object of a triple.
%
holds(S,Chain,O) ?>
  { is_list(Chain),! },
  holds_chain(S,Chain,O).

holds(S,P,O) ?>
  { once((var(P); is_object_property(P))) },
  { strip_operator_(O,O1,=) },
  holds_object(S,P,O1),
  % TODO: check is_object_property(P) in case it was a var before.
  % limit result count to 1 if property is functional
  % TODO: this is problematic because inconsistent assertions
  %          cannot be retrieved. But at least reasoning could
  %          be disabled in case some value could be received from DB.
  % TODO: also handle functional for datatype properties, they can also
  %           be tagged as such
  { is_functional_property(P) -> ! ; true }.

holds(S,P,DataTerm) ?>
  { once((var(P); is_data_property(P))) },
  % data values can be wrapped in operators as in *<(5.0)*
  { strip_operator_(DataTerm,StrippedQuery,QueryOperator) },
  % data values can be wrapped in units as in *kg(5.0)*
  { data_term_unit_(StrippedQuery,QueryValue,QueryUnit) },
  % ground DataValue with requested value in case no particular
  % data unit was requested.
  { ground(QueryUnit) ->
    % XXX need to apply the operator after retrieval
    %       in case of unit conversion.
    % TODO: better convert request to unit in which data is stored
    %       before querying! would need a kind of schema.
    % FIXME basetype missing in second case!
    ( ValueQuery=..[=,            unit(double(DataValue),DataUnit)] );
    ( ValueQuery=..[QueryOperator,unit(DataValue,DataUnit)],
      DataValue=QueryValue )
  },
  holds_data(S,P,ValueQuery),
  % finally ground remaining vars in StrippedQuery
  { once((
      ground(StrippedQuery)
      ; data_term_unify_(QueryOperator,StrippedQuery,DataValue,DataUnit)
    ))
  }.

holds(S,P,O) +>
  { is_object_property(P), ! },
  % O1 should be the bare IRI atom
  { strip_operator_(O,O1,=), atom(O1) },
  triple(S,P,string(O1)).

holds(S,P,DataTerm) +>
  { is_data_property(P), ! },
  { strip_operator_(DataTerm,DataTerm1,=) },
  % extract value and unit from given data term
  { data_term_unit_(DataTerm1,DataValue,DataUnit) },
  % infer the base type of this property (e.g. float, string, term, etc.)
  { data_base_type_(P,DataValue,BaseType) },
  %%
  { Term=..[BaseType,DataValue] },
  holds_data(S,P,unit(Term,DataUnit)).

holds(_,P,_) +>
  { print_message(error, holds(unknown_property(P))) },
  { fail }.

%%
% DB retrieval
holds_object(S,P,O) ?> triple(S,P,O).

%%
% DB retrieval/retrieval
holds_data(S,P,O) ?+> triple(S,P,O).
% OBDA
holds_data(S,P,O) ?> access(holds(S,P,O)).

%%
holds_chain(S,[],O)      ?> { !, S=O }.
holds_chain(S,[P0|Ps],O) ?>
  holds(S,P0,X),
  holds_chain(X,Ps,O).

%%
% Unify requested value with the one that was actually retrieved.
%
data_term_unify_('=',Query,DataValue,DataUnit) :-
  var(Query),!,
  ( ground(DataUnit) ->
    Query=..[DataUnit,DataValue];
    Query=DataValue
  ).

data_term_unify_(Operator,Query,DataValue,DataUnit) :-
  compound(Query),!,
  % TODO: use standard SI unit if var(DataUnit)
  ground(DataUnit),
  Query=..[QueryUnit,QueryValue],
  ( QueryUnit=DataUnit ->
    Actual=DataValue;
    % try to convert units
    data_value_convert_(DataUnit,QueryUnit,
                        DataValue,Actual)
  ),
  Goal=..[Operator,Actual,QueryValue],
  call(Goal).

data_term_unify_('=',DataValue,DataValue,_).

%%
% Extract operator from value term.
% Note that operators can be defined externaly, so X here may
% still be a compound afterwards (e.g. in case of units, or OWL operators
% such as only).
%
strip_operator_(X,     X, =) :- var(X), !.
strip_operator_(=(X),  X, Operator) :- !, Operator='='.
strip_operator_(>(X),  X, Operator) :- !, Operator='>'.
strip_operator_(<(X),  X, Operator) :- !, Operator='<'.
strip_operator_(=<(X), X, Operator) :- !, Operator='=<'.
strip_operator_(>=(X), X, Operator) :- !, Operator='>='.
strip_operator_(X,     X, =).

%%
% Extract unit from value term.
%
data_term_unit_(DataTerm,_,_) :- var(DataTerm), !.
data_term_unit_(unit(DataValue,DataUnit),DataValue,DataUnit) :- !.
data_term_unit_(DataTerm,DataValue,DataUnit) :-
  compound(DataTerm), !,
  DataTerm =.. [DataUnit,DataValue],
  qudt_unit(DataUnit,_,_,_).
data_term_unit_(DataValue,DataValue,_).

%%
% Convert retrieved value to requested unit.
%
data_value_convert_(Unit,Unit,Value0,Value1) :-
  % units are the same, no conversion needed
  !, Value1=Value0.

data_value_convert_(Unit0,Unit1,Value0,Value1) :-
  % units are different
  ground([Unit0,Unit1]),
  qudt_conv(Unit0,Unit1,Value0,Value1).

%%
% Determine the base type of date values (i.e. string,int,foat,etc.).
%
data_base_type_(_Property,DataValue,term) :- compound(DataValue),!.
data_base_type_(Property,_DataValue,XSDType) :-
  has_range(Property,Range),
  xsd_data_basetype(Range,XSDType),!.
data_base_type_(_Property,DataValue,number) :- number(DataValue),!.
data_base_type_(_Property,DataValue,string) :- atom(DataValue),!.
data_base_type_(_Property,DataValue,string) :- string(DataValue),!.
data_base_type_(_Property,true,bool) :- !.
data_base_type_(_Property,false,bool) :- !.
