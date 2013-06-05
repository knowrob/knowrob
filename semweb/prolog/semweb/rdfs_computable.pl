:- module(rdfs_computable,
    [
      rdfs_computable_class/2,       % is the specified class computable?
      rdfs_computable_instance_of/2, % query computable instances of a class
      rdfs_computable_property/2,    % is the specified property computable?
      rdfs_computable_triple/3,      % calculate the value of computable properties
      rdfs_instance_of/2,            % combine rdfs_computable_instance_of (with subclass handling) and rdfs:rdfs_individual_of
      rdf_triple/3,                  % combine rdfs_computable_triple with rdf:rdf_has
      rdfs_assert_prop_conc/2,       % assert property concatenations in a row
      rdfs_assert_prop_conc/3,
      rdf_common_ancestor/2
    ]).
:- use_module(rdfs).
:- use_module(rdf_db).
:- use_module(library(odbc)).
:- use_module(library(lists)).

:- rdf_register_ns(computable, 'http://ias.cs.tum.edu/kb/computable.owl#').
:- rdf_meta
  rdfs_computable_class(t,t),
  rdfs_computable_instance_of(t,t),
  rdfs_computable_property(t,t),
  rdfs_computable_triple(t,t,t),
  rdfs_instance_of(t,t),
  rdf_triple(t,t,t),
  rdfs_assert_prop_conc(t,-),
  rdfs_assert(t,-,?),
  rdf_common_ancestor(t,r).

:- dynamic user:rdf_triple_hook/3.
:- multifile user:rdf_triple_hook/3.



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Query predicates
%


%% rdf_triple(?Property, ?Frame, ?Value) is nondet.
%
% Hook entry that computes triples:
% (rdfs_computable_compute_property_concatenation, rdf_has, rdfs_computable_triple).
%
rdf_triple(Property, Frame, Value) :-

  findall(SubProp, rdfs:rdfs_subproperty_of(SubProp, Property), SubProperties),
  member(SubProperty, SubProperties),

  ( (findall([Frame,Value], rdf_has(Frame, SubProperty, Value), Res),
     member(R, Res),nth0(0, R, Frame), nth0(1, R, Value) )
  ; rdfs_computable_compute_property_concatenation(SubProperty, Frame, Value)
  ; catch( rdfs_computable_triple(SubProperty, Frame, Value), error(instantiation_error, _), fail)
  ; user:rdf_triple_hook(SubProperty, Frame, Value)
  ).

:- user:expand_term((rdf_triple_hook(Property, Frame, Value):-
    rdf_equal(Property, rdfs:range),
    rdfs_instance_of(Frame, computable:'PropertyConcatenation'),
    rdf_has(Frame, computable:'rest', RestProperty),
    rdf_triple(rdfs:range, RestProperty, Value)), X),
  assert(user:(X)).
:- user:expand_term((rdf_triple_hook(Property, Frame, Value):-
    rdf_equal(Property, rdfs:range),
    rdfs_instance_of(Frame, am:'Feature'),
    rdf_has(Frame, computable:'rest', RestProperty),
    rdf_triple(rdfs:range, RestProperty, Value)), X),
  assert(user:(X)).
:- user:expand_term((rdf_triple_hook(Property, Frame, Value):-
    rdf_equal(Property, rdfs:domain),
    rdfs_instance_of(Frame, computable:'PropertyConcatenation'),
    rdf_has(Frame, computable:'first', FirstProperty),
    rdf_triple(rdfs:domain, FirstProperty, Value)), X),
  assert(user:(X)).

:- user:expand_term((rdf_triple_hook(Property, Frame, Value):-
    rdf_equal(Property, rdf:type),
    (nonvar(Frame); nonvar(Value)),
    rdfs_instance_of(Frame, Value))
  , X), assert(user:(X)).

:- rdf_meta
   rdf_lineage_class_hierarchy(r, ?),
   rdf_class_compare(?, r, r).



%% rdfs_instance_of(?Resource, ?Class) is nondet.
%
% combine rdfs_computable_instance_of (with subclass handling) and rdfs:rdfs_individual_of
%
rdfs_instance_of(Resource, Class) :-

  nonvar(Resource)

  -> ( nonvar(Class)

       % check if Resource belongs to Class
       -> ((rdfs_individual_of(Resource, Class) ;
            rdfs_computable_instance_of_subclass(Resource, Class)),!)

       % compute the class of the given instance
       ;  (findall(MyClass, (rdfs_individual_of(Resource, MyClass)), Classes),
           member(Class, Classes)) )

  ; (nonvar(Class)    % compute all instances of this class
    -> ( findall(MyResource, (rdfs_individual_of(MyResource, Class);
                              rdfs_computable_instance_of_subclass(MyResource, Class)), Resources),
         member(Resource, Resources))).





% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Find computable definitions
%

%% rdfs_computable_class(+Class, -ComputableClass) is nondet.
%
% Search for computable classes with the target Class
%
rdfs_computable_class(Class, ComputableClass) :-

  rdfs_computable_sql_class(Class, ComputableClass);
  rdfs_computable_prolog_class(Class, ComputableClass).


%% rdfs_computable_sql_class(+Class, -ComputableClass) is nondet.
%
% Implementation of rdfs_computable_class for SqlClasses.
%
rdfs_computable_sql_class(Class, ComputableClass) :-

  findall(CC, rdf_has(CC,computable:'target',Class), ComputableClasses),
  member(ComputableClass, ComputableClasses),
  once(rdfs_individual_of(ComputableClass,computable:'SqlClass')).


%% rdfs_computable_prolog_class(+Class, -ComputableClass) is nondet.
%
% Implementation of rdfs_computable_class for PrologClasses.
%
rdfs_computable_prolog_class(Class, ComputableClass) :-

  findall(CC, rdf_has(CC,computable:'target',Class), ComputableClasses),
  member(ComputableClass, ComputableClasses),
  once(rdfs_individual_of(ComputableClass,computable:'PrologClass')).




%% rdfs_computable_property(+Property, -ComputableProperty) is nondet.
%
% Search ComputableProperty with the target Property.
%
rdfs_computable_property(Property, ComputableProperty) :-
  rdfs_computable_sql_property(Property, ComputableProperty)
  ; rdfs_computable_prolog_property(Property, ComputableProperty).


%% rdfs_computable_sql_property(+Property, -ComputableProperty) is nondet.
%
% Implementation of rdfs_computable_Property for SQLProperties.
%
rdfs_computable_sql_property(Property, ComputableProperty) :-

  findall(CP, rdf_has(CP,computable:'target',Property), ComputableProperties),
  member(ComputableProperty, ComputableProperties),
  once(rdfs_individual_of(ComputableProperty,computable:'SqlProperty')).


%% rdfs_computable_prolog_property(+Property, -ComputableProperty) is nondet.
%
% Implementation of rdfs_computable_Property for PrologProperties.
%
rdfs_computable_prolog_property(Property, ComputableProperty) :-

  findall(CP, rdf_has(CP,computable:'target',Property), ComputableProperties),
  member(ComputableProperty, ComputableProperties),
  once(rdfs_individual_of(ComputableProperty,computable:'PrologProperty')).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Perform computation (caching, call the different types of computables)
%



%% rdfs_computable_instance_of_subclass(?Instance, ?Class) is nondet.
%
% Compute computable instances of all sub-classes of Class
%
rdfs_computable_instance_of_subclass(Resource, Class) :-
  nonvar(Class), !,
  findall(SC, rdfs_subclass_of(SC, Class), SCs),
  member(SubClass, SCs),
  rdfs_computable_instance_of(Resource, SubClass).
rdfs_computable_instance_of_subclass(Resource, Class) :-
  nonvar(Resource), !,
  findall(SC, rdfs_subclass_of(SC, Class), SCs),
  member(SubClass, SCs),
  rdfs_subclass_of(SubClass, Class).
rdfs_computable_instance_of_subclass(_,_) :-  % both are Variables
  throw(error(instantiation_error, _)).


%% rdfs_computable_instance_of(?Instance, ?Class) is nondet.
%
% Compute computable instances of Class
%
% At least one of Instance and Class needs to be nonvar.
%
rdfs_computable_instance_of(Instance, Class) :-
  var(Class),
  var(Instance),!,
  throw(error(instantiation_error, _)).

% compute instances
rdfs_computable_instance_of(Instance, Class) :-
  nonvar(Class),
  var(Instance),
  ((findall(I, rdfs_computable_sql_instance_of(I,    Class), Is), member(Instance, Is)) ;
   (findall(I, rdfs_computable_prolog_instance_of(I, Class), Is), member(Instance, Is))).

% determine classes
rdfs_computable_instance_of(Instance, Class) :-
  var(Class),
  nonvar(Instance),
  ((findall(C, rdfs_computable_sql_instance_of(Instance,    C), Cs), member(Class, Cs));
   (findall(C, rdfs_computable_prolog_instance_of(Instance, C), Cs), member(Class, Cs))).

% check type
rdfs_computable_instance_of(Instance, Class) :-
  nonvar(Class),
  nonvar(Instance),
  ((findall(C, rdfs_computable_sql_instance_of(Instance,    C), Cs), member(Cls, Cs), rdfs_subclass_of(Cls, Class));
   (findall(C, rdfs_computable_prolog_instance_of(Instance, C), Cs), member(Cls, Cs), rdfs_subclass_of(Cls, Class))).



%% rdfs_computable_triple(+Property, ?Frame, ?Value)
%
% Unify the property triple for a computable property.
% Full caching is enabled for everything but (+,-,-).
%
%
% TODO: check where findall could make sense
%
rdfs_computable_triple(Property, _, _) :-
  var(Property),!,
  throw(error(instantiation_error, _)).
rdfs_computable_triple(Property, Frame, Value) :-
  nonvar(Frame)

  -> ( nonvar(Value)  % both frame and value bound:
    -> ( rdf(Property, Frame, Value, cache)
      -> true
      ; ( rdfs_computable_triple_1(Property, Frame, Value),
          ((rdf_has(CP, computable:target, Property), rdf_has(CP, computable:cache, literal(type('http://www.w3.org/2001/XMLSchema#string', cache))))
          -> rdf_assert(Property, Frame, Value, cache)
          ; true)))

    ; ( % frame bound, value unbound
      rdf(computable:cachedAllValuesFor, Property, Frame, cache)

      -> % return cached values
        setof(MyValue, rdf(Property, Frame, MyValue, cache), Values),
        member(Value, Values)

      ; % compute values and cache them
        setof(MyValue, rdfs_computable_triple_1(Property, Frame, MyValue), Values),
        ( (rdf_has(CP, computable:target, Property), rdf_has(CP, computable:cache, literal(type('http://www.w3.org/2001/XMLSchema#string', cache))))
          -> rdf_assert(computable:cachedAllValuesFor, Property, Frame, cache),
             maplist(rdfs_computable_cache_values(Property, Frame), Values)
          ; true),
        member(Value, Values)
      ))
  ; nonvar(Value)
    -> ( % frame unbound, value bound
        rdf(Property, Value, computable:cachedAllFramesFor, cache)

      -> setof(MyFrame, rdf(Property, MyFrame, Value, cache), Frames),
        member(Frame, Frames)

      ; setof(MyFrame, rdfs_computable_triple_1(Property, MyFrame, Value), Frames),
        ( (rdf_has(CP, computable:target, Property), rdf_has(CP, computable:cache, literal(type('http://www.w3.org/2001/XMLSchema#string', cache))))
          -> rdf_assert(Property, Value, computable:cachedAllFramesFor, cache),
             maplist(rdfs_computable_cache_frames(Property, Value), Frames)
          ; true),
        member(Frame, Frames))
    ; % both frame and value unbound -> no caching
      rdfs_computable_triple_1(Property, Frame, Value).

% The real work is done here
rdfs_computable_triple_1(Property, Frame, Value) :-
    catch(rdfs_computable_sql_triple(Property,    Frame, Value), error(instantiation_error, _), fail)
  ; catch(rdfs_computable_prolog_triple(Property, Frame, Value), error(instantiation_error, _), fail).


% Helpers for caching...
rdfs_computable_cache_values(Property, Frame, Value) :- rdf_assert(Property, Frame, Value, cache).
rdfs_computable_cache_frames(Property, Value, Frame) :- rdf_assert(Property, Frame, Value, cache).









% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Implementation: Computable classes
%


%%
% rdfs_computable_sql_instance_of(?Instance, ?Class).
%
% Implementation of rdfs_computable_instance_of for SqlClasses.
%
% Either Instance or Class has to be a nonvar.
% This makes almost no sense to do in a both-var-case because
% in our domain we have far too many individuals to consider all
% of them at once. So restrictions should be done before.
%
rdfs_computable_sql_instance_of(Instance, Class) :-

  nonvar(Class),!,
  atom(Class),
  rdf_split_url(Namespace, _, Class),

  % find computable classes
  rdfs_computable_sql_class(Class, ComputableClass),

  % read the command
  ( var(Instance)
  -> rdf_has(ComputableClass,computable:'command',literal(type(_,Command)))
  ;  rdf_has(ComputableClass,computable:'testCommand',literal(type(_,TestCommand))),
       atom(Instance),
       rdf_split_url(Namespace, MyInstance, Instance),
       concat_atom(TestCommandList, '~instance~', TestCommand),
       concat_atom(['\'', MyInstance, '\''], InstanceStr),
       concat_atom(TestCommandList, InstanceStr, Command)
  ),

  % send command to the DB
  rdf_has(ComputableClass,computable:'user',literal(type(_,User))),
  rdf_has(ComputableClass,computable:'password',literal(type(_,PASSWD))),
  rdf_has(ComputableClass,computable:'database',literal(type(_,DB))),
  odbc_connect(DB,Connection,[user(User),password(PASSWD),open(once)]),

  ( var(Instance)
  -> odbc_query(Connection, Command, row(MyInstance)),
      % create the instance in the KB
      rdf_split_url(Namespace, MyInstance, Instance),
      rdf_assert(Instance, rdf:type, Class)
   ; odbc_query(Connection, Command, row(MyInstance))).



rdfs_computable_sql_instance_of(Instance, Class) :-

  nonvar(Instance),!,
  atom(Instance),
  rdf_split_url(_, MyInstance, Instance),

  % find computable classes
  rdfs_computable_sql_class(Class, ComputableClass),

  % read the command
  rdf_has(ComputableClass,computable:'testCommand',literal(type(_,TestCommand))),
  concat_atom(TestCommandList, '~instance~', TestCommand),
  concat_atom(['\'', MyInstance, '\''], InstanceStr),
  concat_atom(TestCommandList, InstanceStr, Command),

  % send command to the DB
  rdf_has(ComputableClass,computable:'user',literal(type(_,User))),
  rdf_has(ComputableClass,computable:'password',literal(type(_,PASSWD))),
  rdf_has(ComputableClass,computable:'database',literal(type(_,DB))),
  odbc_connect(DB,Connection,[user(User),password(PASSWD),open(once)]),
  odbc_query(Connection, Command, row(MyInstance)).



%% rdfs_computable_prolog_instance_of(?Instance, ?Class).
%
% Implementation of rdfs_computable_instance_of for PrologClasses.
%
%
rdfs_computable_prolog_instance_of(Instance, Class) :-

%  \+ (rdf_has(Instance, rdf:type, Class)),

  % get the associated prolog computable
  rdfs_computable_prolog_class(Class, ComputableClass),

  % get the Prolog predicate that is used for evaluation:
  rdf_has(ComputableClass, computable:command, literal(type(_, Cmd))),

  % handle the case that the predicate is defined in another module
  ((term_to_atom(Module:Pred, Cmd)) ->
    (Command=Module:Pred) ;
    (Command=Cmd)),

  (
    (nonvar(Instance))
    ->  call(Command, Instance, Class);
  (

    call(Command, MyInstance, Class),

    % check if MyInstance is already a global RDF URI
    ((rdf_split_url('', _, MyInstance)) -> (
      rdf_split_url(Namespace, _, Class),
      rdf_split_url(Namespace, MyInstance, Instance),
      rdf_assert(Instance, rdf:type, Class)
    );(
      Instance=MyInstance,
      rdf_assert(Instance, rdf:type, Class)
    ))
     )).

%     % result: PrologValue
%     ( PrologValue=[_|_]
%     -> member(Temp, PrologValue),
%       rdfs_prolog_to_rdf(Property, Temp, Instance)
%     ; PrologValue=[]
%     -> fail
%     ; rdfs_prolog_to_rdf(Property, PrologValue, Instance)))).




% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Implementation: Computable properties
%


%% rdfs_computable_sql_triple(Property, Frame, Value).
%
% SQLProperty implementation for rdfs_computable_triple(Property, Frame, Value)
%
rdfs_computable_sql_triple(Property, Frame, Value) :-
  rdfs_computable_sql_property(Property, ComputableProperty),
  atom(Property),
  rdf_split_url(Namespace, _, Property),
  ( nonvar(Frame)
  -> rdf_has(ComputableProperty,computable:'valueSelect',literal(type(_,FrameCommand))),
      atom(Frame),
      rdf_split_url(_, MyFrame, Frame),
      concat_atom(FrameCommandList, '~frame~', FrameCommand),
      concat_atom(FrameCommandList, MyFrame, Command),
      Row = row(X)
  ; nonvar(Value)
  ->  rdf_has(ComputableProperty,computable:'frameSelect',literal(type(_,ValueCommand))),
      rdfs_computable_value_from_url(Property, Value, MyValue),
      concat_atom(ValueCommandList, '~value~', ValueCommand),
      concat_atom(ValueCommandList, MyValue, Command),
      Row = row(MyFrame)
  ;   rdf_has(ComputableProperty,computable:'frameValueSelect',literal(type(_,Command))),
      Row = row(MyFrame,X)
  ),
  rdf_has(ComputableProperty,computable:'user',literal(type(_,User))),
  rdf_has(ComputableProperty,computable:'password',literal(type(_,PASSWD))),
  rdf_has(ComputableProperty,computable:'database',literal(type(_,DB))),
  odbc_connect(DB,Connection,[user(User),password(PASSWD),open(once)]),
  odbc_query(Connection, Command, Row),
  (nonvar(Value), nonvar(Frame)
  -> rdfs_computable_value_from_url(Property, Value, X)
  ; true),
  (var(Value)
  -> rdfs_computable_value_from_url(Property, PrologValue, X),
    rdfs_prolog_to_rdf(Property, PrologValue, Value)
  ; true),
  (var(Frame)
  -> rdf_split_url(Namespace, MyFrame, PrologFrame),
    rdfs_prolog_to_rdf(Property, PrologFrame, Frame)
  ; true).


%% rdfs_computable_prolog_triple(?Property, ?Frame, ?Value).
%
% Evaluation of RDF triples using Prolog
%
rdfs_computable_prolog_triple(Property, Frame, Value) :-
  nonvar(Property),

  % get the associated prolog computable
  rdfs_computable_prolog_property(Property, ComputableProperty),

  % get the Prolog predicate that is used for evaluation:
  rdf_has(ComputableProperty, computable:command, literal(type(_, Cmd))),

  % handle the case that the predicate is defined in another module
  ((term_to_atom(Module:Pred, Cmd)) ->
    (Command=Module:Pred) ;
    (Command=user:Cmd)),

  % execute the Prolog predicate (namespace expansion etc.)
  (
    (nonvar(Value)) ->
      (call(Command, Frame, Value))
    ;
      (call(Command, Frame, PrologValue),

        % result: PrologValue
        (PrologValue=[_|_]
        -> member(Temp, PrologValue),
          rdfs_prolog_to_rdf(Property, Temp, Value)
        ; PrologValue=[]
        -> fail
        ; rdfs_prolog_to_rdf(Property, PrologValue, Value))
      )
  ).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Helper predicates: Convert results
%


%
% Convert between Prolog and RDF values (i.e. name spaces, assert types etc)
%
%
rdfs_prolog_to_rdf(Property, PrologValue, RDFValue) :-
  rdf_has(Property, rdfs:range, Range),
  ((rdfs_individual_of(Range, rdf:'Class')
    ; rdfs_individual_of(Range, owl:'Class'))
  -> %RDFValue = PrologValue

     % add namespace if not present yet

    ((\+ atom(PrologValue) -> term_to_atom(PrologValue, PrologValueTerm); (PrologValueTerm = PrologValue))),

     ( (rdf_split_url('', _, PrologValueTerm)) ->
       (rdf_split_url(NS, _, Range),
       atom_concat(NS, PrologValue, RDFValue),
       !,rdf_assert(RDFValue, rdf:type, Range));
      (RDFValue = PrologValue))

  ; rdf_global_id(NS:_, Range),
    NS = xsd
  -> RDFValue = literal(type(Range, PrologValue))
  ; rdf_has(Range, rdf:type, owl:'DataRange')
  -> (rdf_has(Range, rdf:oneOf, OneOf),
    rdfs_list_to_prolog_list(OneOf, OneOfList),
    member(literal(type(Type, PrologValue)), OneOfList)
    -> RDFValue = literal(type(Type, PrologValue))
    ; member(PrologValue, OneOfList)
    -> RDFValue = PrologValue
    ; print(['The value ',PrologValue,' is not in the oneOf range list ',OneOfList]), fail)
  ; print(['Cannot determine the type for ', PrologValue, ' in the range ', Range])).


%%
% rdfs_computable_value_from_url(+Property, ?Value, ?MyValue) :-
%
% Extract the value from an url depending on the property.
% If property is a owl:DatatypeProperty do no conversion else do a rdf_split split.
%%
rdfs_computable_value_from_url(Property, Value, MyValue) :-
  rdf_has(Property, rdf:type, owl:'DatatypeProperty')
  -> MyValue = Value
  ; ( atom(Property), atom(Value),
     rdf_split_url(Namespace, _, Property),
     rdf_split_url(Namespace, MyValue, Value), !)
  ; MyValue = Value.




% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Helper predicates: Property concatenations
%

%%
% Create a propertyConcatenation from a prolog list
%
rdfs_assert_prop_conc(List, Ressource) :-
  rdfs_assert_prop_conc(List, Ressource, user).
rdfs_assert_prop_conc([], _, _) :- !,
  throw(error(not_enough_arguments_in_proc_conc_list, _)).
rdfs_assert_prop_conc([_], _, _) :- !,
  throw(error(not_enough_arguments_in_proc_conc_list, _)).
rdfs_assert_prop_conc([A,B], List, DB) :- !,
  rdf_bnode(List),
  rdf_assert(List, computable:rest, B, DB),
  rdf_assert(List, computable:first, A, DB),
  rdf_assert(List, rdf:type, computable:'PropertyConcatenation', DB).
rdfs_assert_prop_conc([A|Rest], List, DB) :- !,
  rdfs_assert_prop_conc(Rest, Tail, DB),
  rdf_bnode(List),
  rdf_assert(List, computable:rest, Tail, DB),
  rdf_assert(List, computable:first, A, DB),
  rdf_assert(List, rdf:type, computable:'PropertyConcatenation', DB).

%%
% Compute the value of a propertyConcatenation
%
rdfs_computable_compute_property_concatenation(Parameter, Frame, ParameterValue) :-
  once(rdfs_individual_of(Parameter, computable:'PropertyConcatenation')),
  rdf_has(Parameter, computable:'first', FirstProperty),
  rdf_has(Parameter, computable:'rest', RestProperty),
  rdf_triple(FirstProperty, Frame, Value),
  rdf_triple(RestProperty, Value, ParameterValue).

% TODO: Allow am:PropertyConcatenation and computable:PropertyCombination
%
% Compute a parameter for a function. This can be any type of Parameter.
% e.g. this can be a ComputableProperty or a PropertyConcatenation.
%
% rdfs_computable_compute_parameter(Parameter, Frame, ParameterValue) :-
%  rdf_triple(Parameter, Frame, ParameterValue).






% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Helper predicates: Class hierarchy
%

%%
% Helper predicates for rdf_common_ancestor/2.
%
rdf_superclass_list([], []).
rdf_superclass_list([C|CRest], [SCs| SCRest]) :-
  findall(SC, rdfs_subclass_of(C, SC), SCs),
  rdf_superclass_list(CRest, SCRest).

intersection_of_sets([], []).
intersection_of_sets([L], L).
intersection_of_sets([L0, L1|LRest], Intersection) :-
  intersection(L0, L1, L),
  intersection_of_sets([L|LRest], Intersection).

most_specific_class([C], C).
most_specific_class([C1,C2|Cs], C) :-
  rdfs_subclass_of(C2, C1)
  -> most_specific_class([C2|Cs], C)
  ; most_specific_class([C1|Cs], C). % Either not comparable or C2 is superclass of C1

%% rdf_common_ancestor(+Classes, ?Ancestor).
%
% Get one of the most specific common ancestors of rdf_classes.
%
rdf_common_ancestor([C], C).
rdf_common_ancestor([C1, C2| Cs], C) :-
  rdf_superclass_list([C1, C2| Cs], SCs),
  intersection_of_sets(SCs, CSCs),
  most_specific_class(CSCs, C0),
  C = C0.

rdf_class_compare(Delta, Class0, Class1) :-
  rdfs_subclass_of(Class0, Class1)
  -> (rdfs_subclass_of(Class1, Class0)
    -> Delta = '='
    ; Delta = '<'
    )
  ; rdfs_subclass_of(Class1, Class0)
  -> Delta = '>'
  ; throw(error(not_comparable,_)).




% unsused?
rdfs_computable_literal_value(Literal, _) :-
  var(Literal), !,
  throw(error(instantiation_error, _)).
rdfs_computable_literal_value(Literal, Value) :-
  nonvar(Literal),
  (Literal=literal(type(_,Value))
  -> true
  ; Value=Literal).