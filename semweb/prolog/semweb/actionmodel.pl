%%
%% Copyright (C) 2009 by Bernhard Kirchlechner
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%
:- module(actionmodel,
    [
    ]).
:- use_module(rdfs).
:- use_module(rdf_db).
:- use_module(rdfs_computable).
:- use_module(owl).
:- use_module(library(jpl)).
:- use_module(library(lists)).
:- use_module(library(util)).

:- multifile user:rdf_triple_hook/3.
:- discontiguous user:rdf_triple_hook/3.
:- dynamic user:rdf_triple_hook/3.

:- rdf_register_ns(actionmodel, 'http://ias.cs.tum.edu/kb/actionmodel.owl#').
:- rdf_register_ns(am, 'http://ias.cs.tum.edu/kb/actionmodel.owl#').
:- rdf_meta
  am_property_name(r,-).
:- discontiguous am_rdf_triple/3.

%% am_property_name(+Property, -PropertyName)
%
% Calculate a property name. PropertyConcatenations are broken down.
%
am_property_name(Property, Name) :-
  atom(Property),
  rdfs_instance_of(Property, computable:'PropertyConcatenation')
  -> ( rdf_triple(computable:first, Property, Property0),
       rdf_triple(computable:rest, Property, Property1),
       am_property_name(Property0, Name0),
       am_property_name(Property1, Name1),
       term_to_atom(Name0-Name1, Name) )
  ; rdf_global_id(NS:Local, Property)
  -> term_to_atom(NS:Local, Name)
  ; term_to_atom(Property, Name).

%% am_get_attributes_for_predicates(+Predicates, +Instances, -Attributes)
%
% Get the attribute corresponding to the property names of the predicates in instances.
%
am_get_attributes_for_predicates([], _, []).
am_get_attributes_for_predicates([Predicate|PRest], Instances, [Attribute|ARest]) :-
	am_property_name(Predicate, Name),
	jpl_call(Instances, attribute, [Name], Attribute),
	am_get_attributes_for_predicates(PRest, Instances, ARest).

%% am_concat_predictables(+Ps, -ConcatedPs).
%
% concat a list of elements with -. Do we really need this?
%
am_concat_predictables([P], P).
am_concat_predictables([P0,P1|Ps], P) :-
  am_concat_predictables([P1|Ps], Px),
  P = P0-Px.

%% am_type_value_pair_in_one_of_list(?Type, ?Value, +OneOfList)
%
% Find all type value pairs in the given oneOfList
%
am_type_value_pair_in_one_of_list(Type, Value, OneOfList) :-
  rdfs_list_to_prolog_list(OneOfList, PrologLiteralList),
  member(literal(type(Type, Value)), PrologLiteralList).

%% am_literal_value(+Type, +Value, -TypedValue)
%
% Create a literal(type(Type, Value)) pair, but look into owl:oneOfs.
%
am_literal_value(Type, Value, literal(type(Type, TermValue))) :-
  (var(TermValue), nonvar(Value)
  -> (atom_to_term(Value, TermValue, []); TermValue=Value)
  ; TermValue=Value),
  rdf_global_id(xsd:_, Type), !.
am_literal_value(Type, Value, literal(type(OneOfType, Value))) :-
  rdf_triple(owl:oneOf, Type, OneOfList),
  am_type_value_pair_in_one_of_list(OneOfType, Value, OneOfList).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CALCULATE ATTRIBUTES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% am_calculate_mining_attributes_for_actions(+Attribute, +Action, +Situation, -Result)
%
% Calculate the mining attribute(s) for the given action/situation(s).
%
am_calculate_mining_attribute_for_action(Attribute, Action, Situation, Result):-
  nonvar(Attribute),
  (rdfs:rdfs_instance_of(Attribute, 'http://ias.cs.tum.edu/kb/actionmodel.owl#Feature')
  -> rdf_triple('http://ias.cs.tum.edu/kb/actionmodel.owl#calculatedBy', Attribute, Predicate)
  ; Predicate = Attribute),
  has_to_work(actionmodel:(rdf_triple('http://www.w3.org/2000/01/rdf-schema#domain', Predicate, Domain),
    rdfs_subclass_of(Domain, 'http://ias.cs.tum.edu/kb/actionmodel.owl#Action')
    -> Frame = Action
    ; rdfs_subclass_of(Domain, 'http://ias.cs.tum.edu/kb/actionmodel.owl#Situation')
    -> Frame = Situation
    ; fail
    ), cannot_use_attribute_for_action_or_situation),
  setof(R, rdf_triple(Predicate, Frame, R), Results),
  ( length(Results, 1)
  -> [Result] = Results
  ; Result = Results).

%% am_calculate_mining_attributes_for_action(Attributes, Action, Situation, ResultList)
%
% Calculate all given mining_attribute properties for one action.
%
am_calculate_mining_attributes_for_action([], _, _, []).
am_calculate_mining_attributes_for_action([Attribute|ARest], Action, Situation, [Result|RRest]):-
  nonvar(Attribute), nonvar(ARest),
  am_calculate_mining_attribute_for_action(Attribute, Action, Situation, Result),
  am_calculate_mining_attributes_for_action(ARest, Action, Situation, RRest).

%% am_calculate_mining_attributes_for_actions(+Attributes, +Actions, +Situations, -Results)
%
% Calculate the given mining_attributes for all given actions/situations
% The result is a list of mining_attributes (i.e. lists) for each action
%
am_calculate_mining_attributes_for_actions(_,[], [], []).
am_calculate_mining_attributes_for_actions(Attributes, [Action|ARest], [Situation|SRest], [Result|RRest]):-
  nonvar(Attributes),
  print_info('.', informational),
  has_to_work(actionmodel:am_calculate_mining_attributes_for_action(Attributes, Action, Situation, Result), unable_to_calculate_attributes),
  am_calculate_mining_attributes_for_actions(Attributes, ARest, SRest, RRest).

%% am_calculate_observables_for_situations(+Obs, +Actions, +Sits, -Result).
%
% Calculate the observable values for the given situations.
%
am_calculate_observables_for_situations(Obs, Actions, Sits, Results) :-
  catch(am_calculate_mining_attributes_for_actions(Obs, Actions, Sits, Results), error(java_exception(A),B),
    (jpl_call(A, printStackTrace, [], _), flush, throw(error(java_exception(A),B)))).
am_calculate_observables_for_situation(Obs, Action, Situation, Results) :-
  am_calculate_mining_attributes_for_action(Obs, Action, Situation, Results).

%% am_calculate_predictables_for_actions(+Predictables, +Actions, +Situations, -Result).
%
% Calculate the predictable values for the given actions and situations.
%
am_calculate_predictables_for_actions(Predictables, Actions, Situations, PValues) :-
  am_calculate_mining_attributes_for_actions(Predictables, Actions, Situations, PValues).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MINING ATTRIBUTES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
am_mining_attributes(Predicates, PredicateValues, Attributes) :-
  am_attributes_nominal_or_numeric(Predicates, NumNom),
  am_mining_attributes_1(Predicates, NumNom, PredicateValues, Attributes, 0).

%% am_mining_attributes(+Observables, +ObsValues, +Predictables, +PValues, -Attributes)
%
% Create a weka.core.FastVector and add all mining attribute information (observables and predictables).
%
am_mining_attributes(Observables, ObsValues, Predictables, PValues, Attributes) :-
  am_attributes_nominal_or_numeric(Observables, NumNom),
  am_mining_attributes_1(Observables, NumNom, ObsValues, OAttributes, 0),
  am_attributes_nominal_or_numeric(Predictables, NumNomP),
  am_mining_attributes_1(Predictables, NumNomP, PValues, PAttributes, 0),
  Attributes=OAttributes-PAttributes.

%% am_mining_attributes_1(+Observables, +NominalNumeric, +Values, +Attributes, +N)
%
% Helper predicate for am_mining_attribtues.
%
% am_mining_attributes_1([], [], _, _, _).
am_mining_attributes_1([], [], _, [], _).
am_mining_attributes_1([O|Os], [Nom|Ns], Values, [Attribute|AttRest], N) :-
  am_property_name(O, Name),
  ( Nom = numeric
    -> jpl_new(class([weka, core], ['Attribute']), [Name], Attribute)
    ; Nom = nominal
    -> ( am_mining_attribute_values(O, Values, N, AttValues),
      list_to_fast_vector(AttValues, AttValuesV),
      jpl_new(class([weka, core], ['Attribute']), [Name, AttValuesV], Attribute) )
    ; throw(error(non_numeric_nominal))
  ),
  N1 is N+1,
  am_mining_attributes_1(Os, Ns, Values, AttRest, N1).

%% am_mining_attribute_values(+Predicate, +Values, +VNo, -AttValues)
%
% Get the possible values for a given predicate. If it cannot be determined a priori use the correct column of the given values.
%
am_mining_attribute_values(Predicate, Values, VNo, AttValues) :-
  ( atom(Predicate),
    rdf_triple(rdfs:range, Predicate, Range),
    am_possible_nominal_values(Range, PossValues) )
  -> AttValues = PossValues
  ; extract_column(Values, VNo, LAValues),
    maplist(am_strip_literal_type, LAValues, AValues),
    sort(AValues, AttValues).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  TYPES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% am_strip_literal_type(+Input,-Output)
%
% Strip the literal(type(..., Value)) and return value if present, else return the original.
% A helper for am_possible_nominal_values.
%
am_strip_literal_type(literal(type(_, Value)), Value) :- !.
am_strip_literal_type(Value, Value).

%% am_possible_predicates_values(+Preds, +PValues, +Column, -Results).
%
% Calculate the possible values for all the predicates. See am_mining_attribute_values/4.
%
am_possible_predicates_values([], _, _, []).
am_possible_predicates_values([Predicate|Ps], PValues, N, [Result|Rs]) :-
  am_mining_attribute_values(Predicate, PValues, N, Result),
  N1 is N+1,
  am_possible_predicates_values(Ps, PValues, N1, Rs).

%% am_possible_nominal_values(+Range, -Values)
%
% Get the possible nominal values for a owl:DataRange -> owl:oneOf.
%
am_possible_nominal_values(T1, Vs) :-
  rdfs_instance_of(T1, owl:'DataRange'),
  rdf_triple(owl:oneOf, T1, Range),
  rdfs_list_to_prolog_list(Range, LVs),
  maplist(am_strip_literal_type, LVs, Vs).

%% am_attribute_nominal_or_numeric(+T1,-T0).
%
% Test if the type is nominal or numeric.
% Possible numeric types: xsd:float, xsd:integer, ..., owl:DataRage with only numeric subtypes.
%
am_attribute_nominal_or_numeric(T1, T0) :-
  ( rdf_global_id(NS:Local, T1),
    NS = xsd ), !,
  (  memberchk(Local, [float, double, decimal, integer, long, int, short, byte,  nonPositiveInteger, negativeInteger, nonNegativeInteger, unsignedLong, positiveInteger, unsignedLong, unsignedInt, unsignedShort, unsignedByte])
  -> T0 = numeric
  ; T0 = nominal ).
am_attribute_nominal_or_numeric(T1, T0) :- % Numeric types are always comparable
  rdfs_instance_of(T1, owl:'DataRange'),
  rdf_triple(owl:oneOf, T1, Range),  % so are enumerations of numerals
  rdfs_list_to_prolog_list(Range, List),
  findall(T, (member(E, List), rdfs_instance_of(E, T), am_attribute_nominal_or_numeric(T, nominal)), Ts),
  Ts = [] % if we have no nominals...
  -> T0 = numeric  % ... everything is numeric ...
  ; T0 = nominal.  % ... else everything is nominal

%% am_attributes_nominal_or_numeric(+T1,-T0).
%
% Test if the given types are nominal or numeric.
% See am_attribute_nominal_or_numeric/2.
%
am_attributes_nominal_or_numeric([],[]).
am_attributes_nominal_or_numeric([O|Os], [T|Ts]) :-
  ( rdfs_instance_of(O, am:'Feature') ->
    rdf_triple(am:calculatedBy, O, Prop)
  ; Prop = O),
  rdf_triple(rdfs:range, Prop, T1),
  am_attribute_nominal_or_numeric(T1, T0),
  T = T0,
  am_attributes_nominal_or_numeric(Os, Ts).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FILLING INSTANCES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
am_instances_for_predicates(Predicates, PredicateValues, Instances) :-
  % create the instances object
  am_mining_attributes(Predicates, PredicateValues, AttributeList),
  list_to_fast_vector(AttributeList, Attributes),
  length(PredicateValues, NumRows),
  jpl_new(class([weka, core], ['Instances']), ['BayesNetLearningSet', Attributes, NumRows], Instances),
  % set the class attribute index to the last attribute
  jpl_call(Attributes, 'size', [], NumAttributes),
  NumAttributesM1 is NumAttributes-1,
  jpl_call(Instances, 'setClassIndex', [NumAttributesM1], _),
  % fill the instances
  has_to_work(actionmodel:am_fill_instances(Predicates, PredicateValues, NumAttributes, Instances),
    unable_to_fill_instances).
  
am_fill_instances(_,[],_,_).
am_fill_instances(Predicates, [PredicateValues|Ps], InstLen, Instances) :-
  jpl_new(class([weka,core],['Instance']), [InstLen], Instance),
  jpl_call(Instance, 'setDataset', [Instances], _),
  am_get_attributes_for_predicates(Predicates, Instances, Attributes),
  am_fill_instance(PredicateValues, [], Attributes, Instance),
  jpl_call(Instances, 'add', [Instance], _),
  am_fill_instances(Predicates, Ps, InstLen, Instances).

%% am_fill_instance(ObservableValues, Predictables, Attributes, Instance)
%
% Fill the observables corresponding to the attributes into the instance.
% Then continue with appending the predictables.
%
am_fill_instance([], [], _, _).
am_fill_instance([ObservableLValue|Os], Predictables, [Attribute|As], Instance) :-
  am_strip_literal_type(ObservableLValue, ObservableValue),
  ( jpl_call(Attribute, 'isNominal', [], @(true))
    -> jpl_call(Instance, 'setValue', [Attribute, ObservableValue], _)
    ; %jpl_new(class([java,lang],['Double']), [ObservableValue], DValue),
      %jpl_call(Instance, 'setValue', [Attribute, DValue], _)
      jpl_call(Instance, 'setValue', [Attribute, ObservableValue], _)
  ),
  am_fill_instance(Os, Predictables, As, Instance).
am_fill_instance([], Predictables, _, Instance) :-
  ( \+ is_list(Predictables)
  ; Predictables = []) % If we have no predictables (e.g. for model evalutaion) we leave it
  -> ( jpl_call(Instance, 'classIndex', [], -1)
     -> true
     ; jpl_call(Instance, 'setClassMissing', [], _))
  ; am_concat_predictables(Predictables, Ps),
  term_to_atom(Ps, PA),
  jpl_call(Instance, 'setClassValue', [PA], _).

%% am_fill_instances(+ObservableValuesList, +PredictableValuesList, +InstLen, +Instances)
%
% Fill the Instances object with instances for the given observables and predictables.
% The Predictables- and ObservablesValuesList should contain a list of
% the predictables/observables for each Instance.
%
am_fill_instances(_, [], _, [], _, _).
am_fill_instances(Observables, [ObservableValues|Os], Predictables, [PredictableValues|Ps], InstLen, Instances) :-
  jpl_new(class([weka,core],['Instance']), [InstLen], Instance),
  jpl_call(Instance, 'setDataset', [Instances], _),
%  jpl_call(Instances, 'enumerateAttributes', [], AttEnum),
%  jpl_enumeration_to_list(AttEnum, Attributes),
%  jpl_call(Instances, 'classAttribute', [], ClassAtt),
  append(ObservableValues, PredictableValues, AllValues),
  append(Observables, Predictables, AllPredicates),
  am_get_attributes_for_predicates(AllPredicates, Instances, AllAttributes),
%  append(Attributes, [ClassAtt], AllAttributes),
  am_fill_instance(AllValues, [], AllAttributes, Instance),
% am_fill_instance(ObservableValues, PredictableValues, Attributes, Instance),
  jpl_call(Instances, 'add', [Instance], _),
  am_fill_instances(Observables, Os, Predictables, Ps, InstLen, Instances).

%% am_instance_to_row(+Attributes, +Instance, -Values)
%
% Get the list of values for the list of attributes of the given instance.
%
am_instance_to_row([], Instance, [Value]) :-
  jpl_call(Instance, 'classValue', [], Value).
am_instance_to_row([Att|As], Instance, [Value|Vs]) :-
  jpl_call(Instance, 'value', [Att], Value),  % converts nominal attributes to their attribute index as a double
  am_instance_to_row(As, Instance, Vs).

am_instance_to_string_row([], _, []).
am_instance_to_string_row([Att|As], Instance, [Value|Vs]) :-
  jpl_call(Instance, 'toString', [Att], Value), 
  am_instance_to_string_row(As, Instance, Vs).

%% am_instances_to_table(+Instances, -Table)
%
% Convert an instances object to a table (list of lists)
%
am_instances_to_table(Instances, Table) :-
  jpl_call(Instances, 'enumerateInstances', [], InstEnum),
  jpl_enumeration_to_list(InstEnum, InstList),
  am_attribute_list_from_instances(Instances, Atts),
  maplist(am_instance_to_row(Atts), InstList, Table).

am_instances_to_string_table(Instances, Table) :-
  jpl_call(Instances, 'enumerateInstances', [], InstEnum),
  jpl_enumeration_to_list(InstEnum, InstList),
  am_attribute_list_from_instances(Instances, Atts),
  maplist(am_instance_to_string_row(Atts), InstList, Table).

%% am_attribute_list_from_instances(+Instances, -AttributeList)
%
% Create a list of attributes inside an Instances object
%
am_attribute_list_from_instances(Instances, AttributeList) :-
  jpl_call(Instances, 'numAttributes', [], NumAttributes),
  NumAttributesM1 is NumAttributes-1,
  numlist(0, NumAttributesM1, AttributeNumbers), % AttributeNumbers=range(NumAttributesM1) in python
  maplist(am_nth_attribute_from_instance(Instances), AttributeNumbers, AttributeList).


%% am_nth_attribute_from_instance(+Instances, +Num, -Attribute)
%
% Get the nth attribute of an instance. Attribute=Instance.attribute(Num).
%
am_nth_attribute_from_instance(Instances, Num, Attribute) :-
  jpl_call(Instances, attribute, [Num], Attribute).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MODEL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
% Diverse access methods for models.
%%
am_model_classifiers(Classifiers-_-_-_-_-_-_, Classifiers).
am_model_observables(_-Observables-_-_-_-_-_, Observables).
am_model_predictables_graph(_-_-PredictablesGraph-_-_-_-_, PredictablesGraph).
am_model_predictables_graph(_-_-_-Predictables-_-_-_, Predictables).
am_model_instances(_-_-_-_-Instances-_-_, Instances).
am_model_attributes(_-_-_-_-Instances-_-_, Attributes) :-
  maplist(am_attribute_list_from_instances,Instances, Attributes).
am_model_bayesnet(_-_-_-_-_-BayesNet-_, BayesNet).
am_model_bayesnet_instances(_-_-_-_-_-_-BayesNetInstances, BayesNetInstances).

%% am_necessary_predictables(+Predictable, -NecessaryPredictables).
% Get the predictables the given predictable am:dependsOn.
%
am_necessary_predictables(Predictable, NecessaryPredictables) :-
  findall(Nec, rdf_triple('http://ias.cs.tum.edu/kb/actionmodel.owl#dependsOn', Predictable, Nec), NecessaryPredictables).

am_copy_attributes(OriginalAttributes, Copies) :-
	length(OriginalAttributes, Length),
	unifill_list([], Length, Params),
	jpl_call_for_each(OriginalAttributes, copy, Params, Copies).

%% am_attribute_list_to_attributes(+PredictableIndexes, +ClassIndex, +OAttributeList_PAttributeList, -Attributes)
% Create a FastVector of Attributes consisting of the ObservableAttributes, the indexed predictable attributes
% and the indexed class attribute.
%
am_attribute_list_to_attributes(PredictableIndexes, N, OAttributeList-PAttributeList, Attributes) :-
	am_copy_attributes(OAttributeList, OAttributes),
	list_to_fast_vector(OAttributes, Attributes),
	append(PredictableIndexes, [N], Indexes),
	extract_values_from_row(Indexes, PAttributeList, PIAttributeList),
	am_copy_attributes(PIAttributeList, PAttributes),
	fast_vector_add_list(Attributes, PAttributes).

%%%%%%%%%%% CLASSIFIER LEARNING %%%%%%%%%%%%%%%%%

%% am_create_classifiers(+Observables, +ObservableValues, +Predictables, +PredictableValues,-Classifiers,-Instances)
%
% Creates the classifiers for the action model.
%
am_create_classifiers(Observables, ObservableValues, Predictables, PredictableValues, Classifiers, Instances) :-
  % Generate the mining attributes
  print_info('\tConstruct mining attributes... ', informational),
  has_to_work(actionmodel:am_mining_attributes(Observables, ObservableValues, Predictables, PredictableValues, OPAttributes),
    unable_to_construct_mining_attributes),
  print_info('done\n', informational),
  am_create_classifiers(Predictables, Observables, ObservableValues, Predictables, PredictableValues,
     OPAttributes, Classifiers, Instances, 0).
am_create_classifiers([],_,_,_,_,_,[],[],_).
am_create_classifiers([Predictable|PRest], Observables, OValues, Predictables, PValues,
    Attributes, [Classifier|CRest], [Instances|IRest], N) :-
  am_necessary_predictables(Predictable, NecessaryPredictables),
  key_to_index(NecessaryPredictables, Predictables, Indexes),
  extract_values(Indexes, PValues, AddOValues),
  % For all rows append the additional values.
  append(Observables, NecessaryPredictables, FinalObservables),
  maplist(append, OValues, AddOValues, FinalOValues),
  extract_values([N], PValues, FinalPValues),
  am_attribute_list_to_attributes(Indexes, N, Attributes, FinalAttributes),

  am_create_classifier(FinalObservables, FinalOValues, [Predictable], FinalPValues, FinalAttributes, Classifier, Instances),

  N1 is N+1,
  am_create_classifiers(PRest, Observables, OValues, Predictables, PValues, Attributes, CRest, IRest, N1).

%% am_create_classifier(+ObservableValues, +PredictableValues, +Attributes, -Classifiers,-Instances)
%
% Creates a classifier for the action model.
%
am_create_classifier(Observables, ObservableValues, Predictables, PredictableValues, Attributes, Classifier, Instances) :- 
  % Create the instances object for weka ...
  % term_to_atom(Frame, PredictionName), we use a default name for the instances. Does it matter anywhere?
  PredictionName = 'ActionModelRelation',
  length(ObservableValues, NumRows),
  print_info('\tCreating instances... ', informational),
  jpl_new(class([weka, core], ['Instances']), [PredictionName, Attributes, NumRows], Instances),
  jpl_call(Attributes, 'size', [], NumAttributes),
  NumAttributesM1 is NumAttributes-1,
  jpl_call(Instances, 'setClassIndex', [NumAttributesM1], _),
  % ... and fill it
  has_to_work(actionmodel:am_fill_instances(Observables, ObservableValues, Predictables, PredictableValues, NumAttributes, Instances), unable_to_fill_instances),

  % Learn the model
  print_info('done\n\tLearning classifier... ', informational),
  jpl_call(Instances, 'classAttribute', [], ClassAttribute),
  (jpl_call(ClassAttribute, 'isNumeric', [], @(true))
  -> jpl_new(class([weka,classifiers,trees],['REPTree']), [], Classifier)
%    ,jpl_call(Classifier, 'setUnpruned', [@(true)], _)
  ; jpl_new(class([weka,classifiers,trees],['J48']), [], Classifier)),
  jpl_call(Classifier, buildClassifier, [Instances], _),
  print_info('done\n', informational).

%%%%%%%% BAYESNET LEARNING %%%%%%%%%%%%%%%
%% am_bayes_initialize_domain(+Attribute, -Domain)
% Initialize the domain for the given attribute.
%
am_bayes_initialize_domain(Attribute, Splits, Domain) :-
  jpl_call(Attribute, isNumeric, [], @(true))
  -> jpl_call(Attribute, name, [], AttributeName),
    get_assoc(AttributeName, Splits, MainSplits),
    jpl_new('[D', MainSplits, SplitArray),
    jpl_new(class([edu,tum,cs,bayesnets,core],['DiscretizationFilter','Default']), [SplitArray],
            DiscretizationFilter),
    jpl_new(class([edu,tum,cs,bayesnets,core],['Discretized']), [DiscretizationFilter], Domain)
  ;	jpl_call(Attribute, enumerateValues, [], ValueEnumeration),
	  jpl_enumeration_to_list(ValueEnumeration, ValueList),
    jpl_datums_to_array(ValueList, ValueArray),
    jpl_new(class([edu,ksu,cis,bnj,ver3,core],['Discrete']), [ValueArray], Domain).


%% am_bayes_initialize_nodes(+BayesNet, +Predicates, +Attributes)
% Initialize the nodes of the bayes net.
%
am_bayes_initialize_nodes(_, [], [], _).
am_bayes_initialize_nodes(BayesNet, [Predicate|Predicates], [Attribute|Attributes], Splits) :-
	am_property_name(Predicate, Name),
	am_bayes_initialize_domain(Attribute, Splits, Domain),
	jpl_call(BayesNet, 'addNode', [Name, Domain], _),
	am_bayes_initialize_nodes(BayesNet, Predicates, Attributes, Splits).

%% am_bayes_initialize_dependency_domain(+ParentName, +PredictableName, +Splits, -Domain)
% Initialize the domain with the dependency dependant splits. This may only be called if the
% parent node is a Discretized node.
%
am_bayes_initialize_dependency_domain(ParentName, PredictableName, Splits, Domain) :-
  has_to_work(get_assoc(ParentName-PredictableName, Splits, DependencySplits),
    cannot_find_dependency_in_splits),
  jpl_new('[D', DependencySplits, SplitArray),
  jpl_new(class([edu,tum,cs,bayesnets,core],['DiscretizationFilter','Default']), [SplitArray], 
          DiscretizationFilter),
  jpl_new(class([edu,tum,cs,bayesnets,core],['Discretized']), [DiscretizationFilter], Domain).
  
%% am_bayes_add_dependencies(+BayesNet, +PredictableName, +Parents)
% Add the connections of the given node of the BayesNet to its parents.
% Additionally rediscretize the domains for discretized nodes by inserting an additional node.
%
am_bayes_add_dependencies(_, _, [], _).
am_bayes_add_dependencies(BayesNet, PredictableName, [Parent|RestParents], Splits) :-
	am_property_name(Parent, ParentName),
%   jpl_call(BayesNet, 'getNode', [ParentName], Node),
%   jpl_call(Node, 'getDomain', [], OldDomain),
% MT: disabled probably unused code to eliminate Prolog warnings (jpl_instance_of undefined)
%   (jpl_instance_of(OldDomain, class([edu,tum,cs,bayesnets,core],['Discretized']))
%   -> has_to_work(actionmodel:am_bayes_initialize_dependency_domain(ParentName, PredictableName, Splits, Domain),
%       unable_to_initialize_dependency_domain),
%     concat_atom([ParentName, PredictableName], '$', NodeName),
%     jpl_call(BayesNet, 'addNode', [NodeName, Domain, ParentName], _),
%     jpl_call(BayesNet, 'connect', [ParentName, NodeName], _),
%     jpl_call(BayesNet, 'connect', [NodeName, PredictableName], _)
%   ;
  jpl_call(BayesNet, 'connect', [ParentName, PredictableName], _),
%   ),
	am_bayes_add_dependencies(BayesNet, PredictableName, RestParents, Splits).
%% am_bayes_initialize_dependencies(BayesNet, Observables, Predictables)
% Initialize the dependencies of the BayesNet.
%
am_bayes_initialize_dependencies(_, _, [], _).
am_bayes_initialize_dependencies(BayesNet, Observables, [Predictable|RestPredictables], Splits) :-
	am_property_name(Predictable, PredictableName),
	am_necessary_predictables(Predictable, NecessaryPredicates),
	append(Observables, NecessaryPredicates, AllParents),
	has_to_work(actionmodel:am_bayes_add_dependencies(BayesNet, PredictableName, AllParents, Splits),
    unable_to_add_dependencies),
	am_bayes_initialize_dependencies(BayesNet, Observables, RestPredictables, Splits).
%% am_bayes_initialize_structure(+BayesNet, +Observables, +Predictables, +Attributes)
% Initialize the structure of the BayesNet: the nodes and their connections.
%
am_bayes_initialize_structure(BayesNet, Observables, Predictables, Attributes, DomainSplits) :-
	append(Observables, Predictables, AllPredicates),
	has_to_work(actionmodel:am_bayes_initialize_nodes(BayesNet, AllPredicates, Attributes, DomainSplits),
    unable_to_initialize_nodes),
	has_to_work(actionmodel:am_bayes_initialize_dependencies(BayesNet, Observables, Predictables, DomainSplits),
    unable_to_initialize_dependencies).

%am_initialize_domain_learner(Attributes, BayesNet, DomainLearner):-
%	am_initialize_domain_learner(Attributes, BayesNet, DomainLearner, [], []).
%am_initialize_domain_learner([], BayesNet, DomainLearner, DirectDomains, ClusteredDomains) :-
%	jpl_datums_to_array(DirectDomains, DDArray),
%	jpl_datums_to_array(ClusteredDomains, CDArray),
%	jpl_new(class([edu,tum,cs,bayesnets,learning],['ClusterNamer','Intervals']), [], ClusterNamer),
%	jpl_new(class([edu,tum,cs,bayesnets,learning],['DomainLearner']), [BayesNet,DDArray,CDArray,ClusterNamer,@(null)], DomainLearner).
%am_initialize_domain_learner([Attribute|RestAttributes], BayesNet, DomainLearner, DirectDomains, ClusteredDomains) :-
%	jpl_call(Attribute, 'name', [], AttributeName),
%	(jpl_call(Attribute, 'isNumeric', [], @(true))
%	-> jpl_new(class([edu,tum,cs,bayesnets,learning],['DomainLearner','ClusteredDomain']), [AttributeName,0], ClusteredDomain),
%	   am_initialize_domain_learner(RestAttributes, BayesNet, DomainLearner, DirectDomains, [ClusteredDomain|ClusteredDomains])
%	; am_initialize_domain_learner(RestAttributes, BayesNet, DomainLearner, [AttributeName|DirectDomains], ClusteredDomains)).

%% am_learn_bayesnet(+BayesNet, +Instances)
%
% Learn the bayesnet for the data given in the WEKA instances.
% Learning is done by CPTLearner via BayesNet.
%
am_learn_bayesnet(BayesNet, Instances) :-
	jpl_new(class([edu,tum,cs,bayesnets,learning],['CPTLearner']), [BayesNet], CPTLearner),
	jpl_call(CPTLearner, 'learn', [Instances], _),
	jpl_call(CPTLearner, 'finish', [], _).

am_attributes([], _, []).
am_attributes([Predicate|Predicates], Instances, [Attribute|Attributes]) :-
	am_property_name(Predicate, Name),
	jpl_call(Instances, attribute, [Name], Attribute),
	am_attributes(Predicates, Instances, Attributes).

am_initialize_filter_cut_points(Filter, [], []) :-
	jpl_call(Filter, setCutPoints,[-1, @(null)], _).
am_initialize_filter_cut_points(Filter, [DomainSplits|DRest], [Attribute|Attributes]) :-
	jpl_call(Attribute, isNumeric, [], @(true))
	-> jpl_call(Attribute, index, [], Index),
	jpl_new('[D', DomainSplits, SplitArray),
	jpl_call(Filter, setCutPoints, [Index, SplitArray], _),
	am_initialize_filter_cut_points(Filter, DRest, Attributes)
	; am_initialize_filter_cut_points(Filter, DRest, Attributes).

%am_convert_instance(_, _, NumRows, NumRows, _).
%am_convert_instance(Instances, Filter, Row, NumRows, NewInstances) :-
%	Row < NumRows,
%	jpl_call(Instances, instance, [Row], Instance),
%	jpl_call(Filter, input, [Instance], @(true)),
%	jpl_call(Filter, output, [], NewInstance),
%	jpl_call(NewInstances, add, [NewInstance], _),
%	NewRow is Row+1,
%	am_convert_instance(Instances, Filter, NewRow, NumRows, NewInstances).
%
%am_convert_instances(Instances, Filter, NewInstances) :-
%	jpl_call(Filter, getOutputFormat, [], OutputFormat),
%	jpl_call(Instances, numInstances, [], NumRows),
%	jpl_new(class([weka, core], ['Instances']), [OutputFormat, NumRows], NewInstances),
%	has_to_work(actionmodel:am_convert_instance(Instances, Filter, 0, NumRows, NewInstances), cannot_convert_instance).

%%
% am_discretize_domains(+Predicates, +Instances, +DomainSplits, -NewInstances).
%
% Use a WEKA DiscretizeFilter to distribute the continuous values to the individual split intervals.
%%
%am_discretize_domains(Predicates, Instances, DomainSplits, NewInstances) :-
%	am_attributes(Predicates, Instances, Attributes),
%	jpl_new(class([weka, filters, supervised, attribute], ['Discretize']), [], Filter), % Create the filter
%	jpl_call(Filter, setInputFormat, [Instances], _), % set the input instances.
%
%	findall(Attribute, (member(Attribute, Attributes), jpl_call(Attribute, isNumeric, [], @(true))), NumericAttributes), % get all numeric attributes...
%	key_to_index(NumericAttributes, Attributes, Indexes), % ... and their indexes ...
%	maplist(plus(1), Indexes, IndexesP1),
%	concat_atom(IndexesP1, ',', IndexesString),
%	jpl_datums_to_array(['-R', IndexesString], Options),
%	jpl_call(Filter, setOptions, [Options], _), % ... to set the range option.
%	
%	am_initialize_filter_cut_points(Filter, DomainSplits, Attributes), % Initialize the filter with the cut points
%%	jpl_call(Filter, isOutputFormatDefined, [], @(true)),
%	am_convert_instances(Instances, Filter, NewInstances).

%% am_create_bayesnet_with_domainsplits(+Observables, +ObservableValues, +Predictables, +PredictableValues, -BayesNet, -Instances, +DomainSplits)
% The domain splits have to be in the same order as [observables|predictables].
% Create a bayesnet from the observable, -values, predictables, -values and domainsplits.
% return the learned bayesnet and the instances.
%
am_create_bayesnet_with_domainsplits(Observables, ObservableValues, Predictables, PredictableValues, BayesNet, Instances, DomainSplits) :-
  % Generate the mining attributes...
  print_info('\tConstruct mining attributes... ', informational),
  length(ObservableValues, NumRows),
  has_to_work(actionmodel:am_mining_attributes(Observables, ObservableValues, Predictables, PredictableValues, OAttributes-PAttributes),
	      unable_to_construct_mining_attributes),
%  append(Observables, Predictables, AllPredicates),
  append(OAttributes, PAttributes, AttributeList),
  list_to_fast_vector(AttributeList, Attributes),
  % ..., the instances ...
  print_info('done\n\tCreating Instances...', informational),
  jpl_new(class([weka, core], ['Instances']), ['BayesNetLearningSet', Attributes, NumRows], Instances),
  jpl_call(Attributes, 'size', [], NumAttributes),
  NumAttributesM1 is NumAttributes-1,
  jpl_call(Instances, 'setClassIndex', [NumAttributesM1], _),
  % ... and fill them.
  has_to_work(actionmodel:am_fill_instances(Observables, ObservableValues, Predictables, PredictableValues, NumAttributes, Instances), unable_to_fill_instances),
%  has_to_work(actionmodel:am_discretize_domains(AllPredicates, Instances, DomainSplits, DiscretizedInstances), % cannot_discretize_domain),
  
  % Then construct the bayesnet...
  print_info('done\n\tConstructing bayesnet...', informational),
  has_to_work(actionmodel:(jpl_new(class([edu,tum,cs,bayesnets,core],['BeliefNetworkEx']), [], BayesNet),
			   am_bayes_initialize_structure(BayesNet, Observables, Predictables, AttributeList, DomainSplits)), unable_to_construct_bayesnet),
  
  % ... and learn it.
  print_info('done\n\tLearning bayesnet...', informational),
  has_to_work(actionmodel:am_learn_bayesnet(BayesNet, Instances), cannot_learn_bayesnet),
  print_info('done', informational).

%% am_create_bayesnet_from_classifiers(+Classifiers, +Instances, +FullInstances, -BayesNet)
%
% Create a bayes net (edu.tum.cs.bayesnets.core.BeliefNetworkEx) from WEKA classifiers using
% edu.tum.cs.bayesnets.learning.BeliefNetworkFromClassifiers via jpl.
% This is preferable to am_create_bayesnet_with_splitpoints because it is easier to implement
% in java and errors can be found much easier that way.
%
am_create_bayesnet_from_classifiers(Classifiers, Instances, FullInstances, BayesNet) :-
  zip(Classifiers, Instances, NCArguments),
  maplist(jpl_new('de.tum.in.fipm.base.util.weka.NamedClassifier'), NCArguments, NamedClassifiers),
  jpl_new('[Lde.tum.in.fipm.base.util.weka.NamedClassifier;', NamedClassifiers, ClassifiersArray),
  jpl_new(class([edu,tum,cs,bayesnets,learning],['BeliefNetworkFromClassifiers']), [ClassifiersArray], BNFC),
  jpl_call(BNFC, checkSplitPoints, [FullInstances], _),
  jpl_call(BNFC, getBeliefNetworkStructure, [], BayesNet),
  % Learn the bayesnet
  jpl_new(class([edu,tum,cs,bayesnets,learning],['CPTLearner']), [BayesNet], CPTLearner),
	jpl_call(CPTLearner, 'learn', [FullInstances], _),
  jpl_call(BNFC, completeCPTs, [BayesNet], _),
	jpl_call(CPTLearner, 'finish', [], _).

%%%%%%%%%% MODEL CONVERSION %%%%%%%%%%%%


%% am_extract_domain_splits_from_trees(+Predicates, +Classifiers, +Instances, -Splits)
% Extract the domain splits used in classifier trees (as e.g. C45 or REPTree) 
% as a list of assocs from parent to child.
%
am_extract_domain_splits_from_trees(Predicates, Classifiers, Instances, Splits) :-
  empty_assoc(Assoc),
  am_extract_domain_splits_from_trees(Predicates, Classifiers, Instances, Assoc, Splits).
am_extract_domain_splits_from_trees([], _, _, Splits, Splits).
am_extract_domain_splits_from_trees([Predicate|PRest], Classifiers, Instances, Acc, Splits) :-
	am_extract_domain_splits_for_predicate(Predicate, Classifiers, Instances, Acc, NewAcc),
	am_extract_domain_splits_from_trees(PRest, Classifiers, Instances, NewAcc, Splits).
  
%% am_extract_domain_splits_for_predicate(+Predicate, +Classifiers, +Instances, +InputAssoc, -Splits)
% Extract the domain splits for the given predicate from the classifier trees.
% InputAssoc is an assoc that probably already stores some splits.
%
am_extract_domain_splits_for_predicate(Predicate, Classifiers, Instances, Assoc, Splits) :-
  am_extract_domain_splits_for_predicate(Predicate, Classifiers, Instances, [], Assoc, Splits).
am_extract_domain_splits_for_predicate(Predicate, [], [], Union, Acc, Splits) :-
  am_property_name(Predicate, PredicateName),
  put_assoc(PredicateName, Acc, Union, Splits). % Push the union of all splits to ground key
am_extract_domain_splits_for_predicate(Predicate, [Classifier|Classifiers], [Instances|IRest], 
      Union, Acc, Splits) :-
	am_get_attributes_for_predicates([Predicate], Instances, [Attribute]),
	(Attribute= @(null)
	-> NewUnion=Union,
    NewAcc=Acc
	; jpl_call(Instances, classAttribute, [], ClassAttr),
    jpl_call(ClassAttr, name, [], ClassAttrName),
    jpl_call(Attribute, index, [], AttrIndex),
    jpl_call(Classifier, getSplitPoints, [AttrIndex], SplitPointsArr),
    jpl_array_to_list(SplitPointsArr, NewSplitPoints),
    list_to_ord_set(NewSplitPoints, NewSplits),
    am_property_name(Predicate, PredicateName),
    put_assoc(PredicateName-ClassAttrName, Acc, NewSplits, NewAcc),
    ord_union(NewSplits, Union, NewUnion)),
	am_extract_domain_splits_for_predicate(Predicate, Classifiers, IRest, NewUnion, NewAcc, Splits).

%%%%%%%%%% ACTION MODEL ACCESS/CREATION %%%%%%%%%

%% am_calculate_action_model(+ActionDefinition, +Observables, +Predictables, -Model)
%
% Calculate a new action model from the given ActionDefinition, Observables and Predictables.
% TODO: Create a model for each predictable and decide what classifier (J48 or RegressionTree) to learn.
%
am_calculate_action_model(Actions, Observables, PredictablesGraph, Model) :-
  % Sorting Predictables
  top_sort(PredictablesGraph, Predictables),
  % Get the actions resp. situations for the ActionDefinition
  print_info('Calculating situations... ', informational),
  has_to_work(findall(Sit,(member(A,Actions), rdf_triple('http://ias.cs.tum.edu/kb/actionmodel.owl#startSituation', A, Sit)),Situations),
    unable_to_retrieve_situations),
  has_to_work(actionmodel:(length(Actions, L), length(Situations, L)), length_of_actions_and_sits_differ),

  % Calculate the observables ...
  print_info('done\nCalculating observables... ', informational),
  has_to_work(actionmodel:am_calculate_observables_for_situations(Observables, Actions, Situations, ObservableValues),
    unable_to_calculate_observables),
  % ... and the predictables for the situations
  print_info('done\nCalculating predictables... ', informational),
  has_to_work(actionmodel:am_calculate_predictables_for_actions(Predictables, Actions, Situations, PredictableValues),
    unable_to_calculate_predictables),

  % Now learn the classifiers
  print_info('done\nCreating bayesnet...\n', informational),
%  has_to_work(actionmodel:am_create_bayesnet(Observables, ObservableValues, Predictables, PredictableValues, Classifiers, Instances),
%	      unable_to_create_bayesnet),
  print_info('done\nCreating classifiers...\n', informational),
  has_to_work(actionmodel:am_create_classifiers(Observables, ObservableValues, Predictables, PredictableValues, Classifiers, Instances),
    unable_to_create_classifiers),
  append(Observables, Predictables, AllPredicates),
  maplist(append, ObservableValues, PredictableValues, AllValues),
%  has_to_work((actionmodel:am_extract_domain_splits_from_trees(AllPredicates, Classifiers, Instances, Domains)), unable_to_extract_splits),
  print_info('done\nCreating bayesnet...\n', informational),
%  has_to_work(actionmodel:am_create_bayesnet_with_domainsplits(Observables, ObservableValues, Predictables, PredictableValues, BayesNet, BayesInstances, Domains),
%	      unable_to_create_bayesnet),
  has_to_work(actionmodel:am_instances_for_predicates(AllPredicates, AllValues, BayesInstances),
    unable_to_create_instances_for_bayesnet),
  has_to_work(actionmodel:am_create_bayesnet_from_classifiers(Classifiers, Instances, BayesInstances, BayesNet),
    unable_to_construct_bayesnet),
  print_info('done\nLearning bayesnet...\n', informational),
%  has_to_work(actionmodel:am_learn_bayesnet(BayesNet, BayesInstances), cannot_learn_bayesnet),
  
  print_info('done\n', informational),

  Model = Classifiers-Observables-PredictablesGraph-Predictables-Instances-BayesNet-BayesInstances.

%% am_get_action_model(+ActionModel, -Model)
%
% Get the action model: If there is a cached one use it or calculate it.
%
am_get_action_model(ActionModel, Model) :-
  % Get the observables, ...
  has_to_work(setof(O,(rdf_triple('http://ias.cs.tum.edu/kb/actionmodel.owl#observable', ActionModel, O)), Observables),
    unable_to_retrieve_observables),
  % ... the predictables ...
  has_to_work(setof(P, (rdf_triple('http://ias.cs.tum.edu/kb/actionmodel.owl#predictable', ActionModel, P)), UnsortedPredictables),
    unable_to_retrieve_predictables),
  % ... create a dependency graph ...
  has_to_work((findall(Parent-Pred, (
      member(Pred, UnsortedPredictables),
      rdfs_instance_of(Pred, 'http://ias.cs.tum.edu/kb/actionmodel.owl#Feature'),
      rdf_triple('http://ias.cs.tum.edu/kb/actionmodel.owl#dependsOn', Pred, Parent)), Edges),
    vertices_edges_to_ugraph(UnsortedPredictables, Edges, PredictablesGraph)), unable_to_create_dependency_graph),
  % ... and sort it ...
  %  has_to_work(top_sort(PredictablesGraph, Predictables), unable_to_sort_predictables),
  % ... and the action definition
  has_to_work(rdf_triple('http://ias.cs.tum.edu/kb/actionmodel.owl#forAction', ActionModel, ActionDefinition), no_forAction),

  % Get the actions
  print_info('Fetching actions... ', informational),
  has_to_work(setof(Action, (owl_individual_of(Action, ActionDefinition)), MyActions),
    unable_to_retrieve_actions),
  print_info('done\n', informational),
%   length(MyActions, NumActions),
%   (NumActions > 200
%   -> length(Actions, 200),
%     append(Actions, _, MyActions)
%   ; Actions = MyActions),
Actions = MyActions,
  (am_asserted_model(Observables, PredictablesGraph, Actions, CModel)
  -> Model = CModel
  ; am_calculate_action_model(Actions, Observables, PredictablesGraph, Model),
    am_assert_model(Model, Actions)).

%% am_asserted_model(+Observables, +Predictables, +Actions, -Model)
%
% Search for an asserted model that fits the given parameters.
%
am_asserted_model(Observables, Predictables, Actions, Model) :-
  rdf_has(CachedObservablesNode, am:'learnedModelObservables', ModelAtom),
  rdfs_list_to_prolog_list(CachedObservablesNode, CachedObservables),
  list_to_set(CachedObservables, CachedObservablesSet),
  list_to_set(Observables, ObservablesSet),
  subtract(ObservablesSet, CachedObservablesSet, []), % Identical Observables?

  rdf_has(CachedPredictablesNode, am:'learnedModelPredictablesGraph', ModelAtom),
  rdfs_list_to_prolog_list(CachedPredictablesNode, CachedPredictablesAsAtoms),
  maplist(atom_to_term,CachedPredictablesAsAtoms, CachedPredictables,_),
  list_to_set(CachedPredictables, CachedPredictablesSet),
  list_to_set(Predictables, PredictablesSet),
  subtract(PredictablesSet, CachedPredictablesSet, []), % Identical Predictables?

  rdf_has(CachedActionsNode, am:'learnedModelActions', ModelAtom),
  rdfs_list_to_prolog_list(CachedActionsNode, CachedActions),
  list_to_set(CachedActions, CachedActionsSet),
  list_to_set(Actions, ActionsSet),
  subtract(ActionsSet, CachedActionsSet, []), % Identical Actions?

  atom_to_term(ModelAtom, Model, []).

%% am_assert_model(+Model, +Actions)
%
% Assert the learned model and the learning instances to the database.
% TODO: This should be done somewhat more like specifying a model.
%
am_assert_model(Classifier-Observables-PredictablesGraph-Predictables-Instances-BayesNet-BayesInstances, Actions) :-
  term_to_atom(Classifier-Observables-PredictablesGraph-Predictables-Instances-BayesNet-BayesInstances, ModelAtom),
  rdf_transaction( (rdfs_assert_list(Observables, ObsNode, models),
    rdf_assert(ObsNode, am:'learnedModelObservables', ModelAtom, models),
    maplist(term_to_atom, PredictablesGraph, PredictablesGraphAsAtoms),
    rdfs_assert_list(PredictablesGraphAsAtoms, PredGraphNode, models),
    rdf_assert(PredGraphNode, am:'learnedModelPredictablesGraph', ModelAtom, models),
    rdfs_assert_list(Predictables, PredNode, models),
    rdf_assert(PredNode, am:'learnedModelPredictables', ModelAtom, models),
    rdfs_assert_list(Actions, ActionNode, models),
    rdf_assert(ActionNode, am:'learnedModelActions', ModelAtom, models) ), models).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ACTION CREATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% am_assign_predictables(+Predictables, +Values, -PredictableValues)
%
% Assign the "-"-separated Values to the corresponding Predictables.
%
am_assign_predictables([Predictable], Value, [[Predictable, Value]]) :-
  !, atom(Value).
am_assign_predictables([Predictable|PRest], Value-ValueRest, [[Predictable, Value]|PVRest]) :-
  am_assign_predictables(PRest, ValueRest, PVRest).

%% am_assert_predicate_values_for_object(+PredicateValues, +Object)
%
% Assert the given predicate values for the object.
%
am_assert_predicate_values_for_object([], _).
am_assert_predicate_values_for_object([[Predicate, Value]|PVRest], Object) :-
  ( rdf_triple(rdf:type, Predicate, am:'Feature')
  -> rdf_triple(am:calculatedBy, Predicate, RealPredicate)
  ; RealPredicate = Predicate),
  rdf_triple(rdfs:range, RealPredicate, Range),!,
  ( rdfs_instance_of(RealPredicate, owl:'DatatypeProperty')
    -> am_literal_value(Range, Value, MyValue)
    ; MyValue = Value ),
  rdf_assert(Object, RealPredicate, MyValue, actions),
  am_assert_predicate_values_for_object(PVRest, Object).


%% am_filter_predicates_domain(+PredicateValues, +Domain, -OutputPredicateValues)
%
% Filter all Predicates that have a specified domain.
% PreidcateValues has to be a list of [preidcate, value] pairs, e.g. [[am:withProbability, 0.5], ...].
%
am_filter_predicates_domain([], _, []) :- !.
am_filter_predicates_domain([[Predicate, Value]|PRest], Domain, [[Predicate, Value]|ORest]) :-
  (rdf_triple(rdf:type, Predicate, am:'Feature')
  -> rdf_triple(am:calculatedBy, Predicate, MyPredicate)
  ; MyPredicate=Predicate),
  rdf_triple(rdfs:domain, MyPredicate, SubDomain),
  rdfs_subclass_of(SubDomain, Domain),!,  % We should never leave a choicepoint reading from rdf...
  am_filter_predicates_domain(PRest, Domain, ORest).
am_filter_predicates_domain([[_,_]|PRest], Domain, OutputPredicates) :-
  am_filter_predicates_domain(PRest, Domain, OutputPredicates).
  
%% am_check_predicates_domain(Predicates, Domain)
%
% Check that the given predicates have the correct domain.
%
am_check_predicates_domain([], _).
am_check_predicates_domain([Predicate|PRest], Domain):-
  (rdf_triple(rdf:type, Predicate, am:'Feature')
  -> rdf_triple(am:calculatedBy, Predicate, MyPredicate)
  ; MyPredicate=Predicate),
  rdf_triple(rdfs:domain, MyPredicate, SubDomain),
  rdfs_subclass_of(SubDomain, Domain),!,  % We should never leave a choicepoint reading from rdf...
  am_check_predicates_domain(PRest, Domain).

%% am_create_action(+PredicateValues, +Situation, +Action, -Action)
%
% Create an action with the given predicate values.
% The action is assert in the rdf_triple database and the reference is returned.
% TODO: We have to transmit all the owlassertions to persistent ones
%   We should refer to the InputAction.
%
am_create_action(PredicateValues, _InputAction, InputSituation, Probability, Action) :-
  % Extract Predicates for the action...
  am_filter_predicates_domain(PredicateValues, 'http://ias.cs.tum.edu/kb/actionmodel.owl#Action', ActionPredicates),
  gensym('action', Action),
  append(ActionPredicates, [['http://ias.cs.tum.edu/kb/actionmodel.owl#withProbability', Probability], ['http://ias.cs.tum.edu/kb/actionmodel.owl#startSituation', InputSituation]], AllPredicateValues),
  am_assert_predicate_values_for_object(AllPredicateValues, Action).

%% am_create_situation(+PredicateValues, +Situation, +Action, -Situation)
%
% Create a situation with the given predicate values.
% The action is assert in the rdf_triple database and the reference is returned.
% TODO: We have to transmit all the owlassertions to persistent ones
%   We should refer to the InputSituation and -Action.
%
am_create_situation(PredicateValues, _InputAction, _InputSituation, Probability, Situation) :-
  % Extract Predicates for the action...
  am_filter_predicates_domain(PredicateValues, 'http://ias.cs.tum.edu/kb/actionmodel.owl#Situation', ActionPredicates),
  gensym('situation', Situation),
  append(ActionPredicates, [['http://ias.cs.tum.edu/kb/actionmodel.owl#withProbability', Probability]], AllPredicateValues),
  am_assert_predicate_values_for_object(AllPredicateValues, Situation).
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MODEL PREDICTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
am_nominal_instance_probability(Classifier, Instance, ClassIndex, Probability) :-
  jpl_call(Classifier, 'distributionForInstance', [Instance], Distribution),
  jpl_array_to_list(Distribution, DistList),
  nth0(ClassIndex, DistList, Probability).

%% am_predict_action_situation_predicates(+Predicates, +Dependencies, +Observables, +ObservableValues, +Attributes, +Instances, +Classifiers, -Values)
% Get the prediction of the corresponding classifier/predictable pair for the given observables/-values.
%
am_predict_action_situation_predicates(Predicates, Dependencies, Observables, ObservableValues, Attributes,
    Instances, Classifiers, ValuesProbAssoc) :-
  empty_assoc(AccumulatorAssoc),
  am_predict_action_situation_predicates(Predicates, Dependencies, Observables, ObservableValues, Attributes,
    Instances, Classifiers, AccumulatorAssoc, ValuesProbAssoc).

am_predict_action_situation_predicates([], [], _, _, [], [], [], ValueProbAssoc, ValueProbAssoc).
am_predict_action_situation_predicates([Predicate|PrRest], [Dependencies|DRest], Observables, ObservableValues,
    [Attributes|ARest], [ModelInstances|IRest], [Classifier|CRest], ValueProbAssoc, ResultAssoc) :-
  % Get the values of the parent predicates from ValueAssoc
  % Due to the ordering of the predicates the ValueAssoc should contain a value for each parent.
  findall(DepValue, (member(Dependency, Dependencies), get_assoc(Dependency, ValueProbAssoc, [DepValue,_])),
    DepValues),
  length(DepValues, NumDepValues), length(Dependencies, NumDependencies), NumDepValues = NumDependencies,

  append(Observables, Dependencies, AllObservables),
  am_get_attributes_for_predicates(AllObservables, ModelInstances, AllAttributes),
  append(ObservableValues, DepValues, AllValues),

  length(Attributes, NumAttributes),
  has_to_work(actionmodel:(jpl_new(class([weka,core], ['Instances']), [ModelInstances, 1], Instances),
    jpl_new(class([weka,core], ['Instance']), [NumAttributes], Instance),
    jpl_call(Instance, 'setDataset', [Instances], _),
    am_fill_instance(AllValues, [], AllAttributes, Instance)), unable_to_fill_dest_instance),
  has_to_work((jpl_call(Classifier, 'classifyInstance', [Instance], DoubleResult),
    jpl_call(Instances, 'classAttribute', [], ClassAttribute)), unable_to_classify_instance),
  (jpl_call(ClassAttribute, 'isNumeric', [], @(true))
    -> MyValue = DoubleResult
    ; IntegerResult is round(DoubleResult),
      am_nominal_instance_probability(Classifier, Instance, IntegerResult, Probability),
      jpl_call(ClassAttribute, 'value', [IntegerResult], MyValue)),
  Value = [MyValue,Probability],
  put_assoc(Predicate, ValueProbAssoc, Value, NewValueAssoc),
  am_predict_action_situation_predicates(PrRest, DRest, Observables, ObservableValues,
    ARest, IRest, CRest, NewValueAssoc, ResultAssoc).

 %%% am_neighbours(Graph, N, Neighbours)
 % Invert order of arg 0 and 1 of neighbours to be usable in maplist.
 %
 am_neighbours(Graph, N, Neighbours) :-
   neighbours(N, Graph, Neighbours).

 special_mul(Mul1, Mul2, Mul1) :- var(Mul2), !.
 special_mul(Mul1, Mul2, Mul2) :- var(Mul1), !.
 special_mul(Mul1, Mul2, Result) :- Result is Mul1*Mul2.


%% am_predict_action_situation_predictables(Model, InputAction, Situation, Action, Probability)
% Predict the list of predictables with the given model.
% The predictables have to exist in the model.
%
am_predict_action_situation_predictables(Model, InputAction, Situation, Predictables, Values, Probabilities) :-
  am_model_observables(Model, Observables),
  am_model_attributes(Model, AttributesList),
  am_model_instances(Model, ModelInstances),
  am_model_predictables_graph(Model, PredictablesGraph),
  am_model_classifiers(Model, Classifiers),

  has_to_work(actionmodel:am_calculate_observables_for_situation(Observables, InputAction, Situation, ObservableValues),
    unable_to_retrieve_dest_observables),

  ugraph_retain_transitive(PredictablesGraph, Predictables, RetainGraph),
  top_sort(RetainGraph, NecessaryPredictables),
  transpose(RetainGraph, DependencyGraph),
  maplist(am_neighbours(DependencyGraph), NecessaryPredictables, DependencyList),

  top_sort(PredictablesGraph, OriginalPredictables),
  zip(Classifiers, AttributesList, ClA),
  zip(ClA, ModelInstances, ClAM),
  zipm(OriginalPredictables, ClAM, PredClassifiers),
  list_to_assoc(PredClassifiers, PredClAMAssoc),
  assoc_extract_value_list(PredClAMAssoc, NecessaryPredictables, FinalClAMs),
  zip(FinalClA, FinalModelInstances, FinalClAMs),
  zip(FinalClassifiers, FinalAttributesList, FinalClA),

  has_to_work(actionmodel:am_predict_action_situation_predicates(NecessaryPredictables, DependencyList,
      Observables, ObservableValues, FinalAttributesList, FinalModelInstances, FinalClassifiers, ValueProbAssoc),
    unable_to_classify_predictables),

  assoc_extract_value_list(ValueProbAssoc, Predictables, ValueProbs),
  zip(Values, Probabilities, ValueProbs).
  
am_make_atom(Atom, Atom):-atom(Atom), !.
am_make_atom(Term, Atom):-
  term_to_atom(Term, Atom).
 
am_predict_action_situation_predictables_bayes(Model, InputAction, Situation, Predictables, Values, Probability) :-
  am_model_observables(Model, Observables),
  am_model_bayesnet(Model, BayesNet),

  has_to_work(actionmodel:am_calculate_observables_for_situation(Observables, InputAction, Situation,
    ObservableValues), unable_to_retrieve_dest_observables),
    
  %% Build the String[][] array for the evidences
  maplist(am_property_name, Observables, ObservableNames),
  maplist(am_strip_literal_type, ObservableValues, ObservableUntypedValues),
  maplist(am_make_atom, ObservableUntypedValues, ObservableValueAtoms),
  zip(ObservableNames, ObservableValueAtoms, ObservablesList),
  maplist(jpl_new('[Ljava.lang.String;'), ObservablesList, ArraysList),
  jpl_new('[[Ljava.lang.String;', ArraysList, Evidences),
  
  maplist(am_property_name, Predictables, PredictableNames),
  jpl_new('[Ljava.lang.String;', PredictableNames, Query),
  
  jpl_call(BayesNet, getAssignmentDistribution, [Evidences, Query, 1000], Samples),
  jpl_array_to_list(Samples, SampleList),
  nth0(0, SampleList, BestSample),
  jpl_get(BestSample, weight, Probability),
  jpl_call(BestSample, getUndiscretizedAssignmentMap, [], Assignments),
  boxed_column(PredictableNames, PredictableNamesArg),
  maplist(jpl_call(Assignments, get), PredictableNamesArg, Values).

%% am_classify_action_situation(+Model, +Action, +Situation, -Action)
%
% Classify a situation and generate an action.
%
am_classify_action_situation(Model, InputAction, InputSituation, Action, Probability) :-
  am_model_predictables_graph(Model, PredictablesGraph),
  top_sort(PredictablesGraph, Predictables),
  has_to_work(actionmodel:am_predict_action_situation_predictables_bayes(Model, InputAction, InputSituation,
    Predictables, Values, Probability), unable_to_classify_predictables),
%  reduce(actionmodel:special_mul, Probabilities, 1, Probability),
  zip(Predictables, Values, PredValues),
  has_to_work(actionmodel:am_create_action(PredValues, InputAction, InputSituation, Probability, Action), unable_to_create_action).
  
%% am_classify_situation_action(+Model, +Action, +Situation, -Situation)
%
% Classify a situation and generate an action.
%
am_classify_situation_action(Model, InputAction, InputSituation, Situation, Probability) :-
  am_model_predictables_graph(Model, PredictablesGraph),
  top_sort(PredictablesGraph, Predictables),
  has_to_work(actionmodel:am_predict_action_situation_predictables_bayes(Model, InputAction, InputSituation,
    Predictables, Values, Probability), unable_to_classify_predictables),
%  reduce(actionmodel:special_mul, Probabilities, 1, Probability),
  zip(Predictables, Values, PredValues),
  has_to_work(actionmodel:am_create_situation(PredValues, InputAction, InputSituation, Probability, Situation), unable_to_create_situation).


%% am_classify_predictable(Model, InputAction, Situation, Predictable, Value, Probability)
% Predict a single predictable value from a model.
% The predictable has to exist in the model.
%
am_classify_predictable(Model, InputAction, Situation, Predictable, Value, Probability) :-
  am_predict_action_situation_predictables(Model, InputAction, Situation, [Predictable], [Value], [Probability]).

%% am_predict_situation_from_observable_values(+Prediction, +ObservableValues, -Situation)
% Predict a situation using the model specified via Prediction using the given ObservableValues
% (used to classify a current observation instead of data read from the DB)
%
am_predict_situation_from_observable_values(Prediction, ObservableValues, Situation) :-

  % Read the actionModel
  has_to_work(rdf_triple('http://ias.cs.tum.edu/kb/actionmodel.owl#byActionmodel', Prediction, ActionModel), no_byActionmodel),

  % Then create the action model
  has_to_work(actionmodel:am_get_action_model(ActionModel, Model), unable_to_create_action_model),

  actionmodel:am_model_predictables_graph(Model, PredictablesGraph),
  top_sort(PredictablesGraph, Predictables),

  actionmodel:am_model_observables(Model, Observables),
  actionmodel:am_model_bayesnet(Model, BayesNet),

    %% Built the String[String[]] array for the evidences
    maplist(actionmodel:am_property_name, Observables, ObservableNames),
    maplist(actionmodel:am_strip_literal_type, ObservableValues, ObservableUntypedValues),
    maplist(actionmodel:am_make_atom, ObservableUntypedValues, ObservableValueAtoms),
    zip(ObservableNames, ObservableValueAtoms, ObservablesList),
    maplist(jpl_new('[Ljava.lang.String;'), ObservablesList, ArraysList),
    jpl_new('[[Ljava.lang.String;', ArraysList, Evidences),

    maplist(actionmodel:am_property_name, Predictables, PredictableNames),
    jpl_new('[Ljava.lang.String;', PredictableNames, Query),

    jpl_call(BayesNet, getAssignmentDistribution, [Evidences, Query, 1000], Samples),
    jpl_array_to_list(Samples, SampleList),
    nth0(0, SampleList, BestSample),
    jpl_get(BestSample, weight, Probability),
    jpl_call(BestSample, getUndiscretizedAssignmentMap, [], Assignments),
    boxed_column(PredictableNames, PredictableNamesArg),
    maplist(jpl_call(Assignments, get), PredictableNamesArg, Values),

  zip(Predictables, Values, PredValues),
  has_to_work(actionmodel:am_create_situation(PredValues, _InputAction, _InputSituation, Probability, Situation), unable_to_create_situation).

%% am_predict_action_from_observable_values(+Prediction, +ObservableValues, -Action)
% Predict an action using the model specified via Prediction using the given ObservableValues
% (used to classify a current observation instead of data read from the DB)
%
am_predict_action_from_observable_values(Prediction, ObservableValues, Action) :-

  % Read the actionModel
  has_to_work(rdf_triple('http://ias.cs.tum.edu/kb/actionmodel.owl#byActionmodel', Prediction, ActionModel), no_byActionmodel),

  % Then create the action model
  has_to_work(actionmodel:am_get_action_model(ActionModel, Model), unable_to_create_action_model),

  actionmodel:am_model_predictables_graph(Model, PredictablesGraph),
  top_sort(PredictablesGraph, Predictables),

    actionmodel:am_model_observables(Model, Observables),
    actionmodel:am_model_bayesnet(Model, BayesNet),

    %% Built the String[String[]] array for the evidences
    maplist(actionmodel:am_property_name, Observables, ObservableNames),
    maplist(actionmodel:am_strip_literal_type, ObservableValues, ObservableUntypedValues),
    maplist(actionmodel:am_make_atom, ObservableUntypedValues, ObservableValueAtoms),
    zip(ObservableNames, ObservableValueAtoms, ObservablesList),
    maplist(jpl_new('[Ljava.lang.String;'), ObservablesList, ArraysList),
    jpl_new('[[Ljava.lang.String;', ArraysList, Evidences),
    
    maplist(actionmodel:am_property_name, Predictables, PredictableNames),
    jpl_new('[Ljava.lang.String;', PredictableNames, Query),
    
    jpl_call(BayesNet, getAssignmentDistribution, [Evidences, Query, 1000], Samples),
    jpl_array_to_list(Samples, SampleList),
    nth0(0, SampleList, BestSample),
    jpl_get(BestSample, weight, Probability),
    jpl_call(BestSample, getUndiscretizedAssignmentMap, [], Assignments),
    boxed_column(PredictableNames, PredictableNamesArg),
    maplist(jpl_call(Assignments, get), PredictableNamesArg, Values),
      
  zip(Predictables, Values, PredValues),
  has_to_work(actionmodel:am_create_action(PredValues, _InputAction, _InputSituation, Probability, Action), unable_to_create_action).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RDF_TRIPLE_HOOK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% am_rdf_triple(actionmodel:'toAction', ?Frame, ?Value).
%
% rdf_triple_hook implementation for am:toAction.
%
am_rdf_triple(Property, Frame, Value):-
  catch(catch(
    ( rdf_equal(Property, 'http://ias.cs.tum.edu/kb/actionmodel.owl#toAction'),
      has_to_work(rdfs_instance_of(Frame, 'http://ias.cs.tum.edu/kb/actionmodel.owl#Prediction'), not_a_prediction_as_frame),
      % Get the actionModel
      has_to_work(rdf_triple('http://ias.cs.tum.edu/kb/actionmodel.owl#byActionmodel', Frame, ActionModel), no_byActionmodel),
    
      % Then calculate the action model
      has_to_work(actionmodel:am_get_action_model(ActionModel, Model), unable_to_create_action_model),
    
      print_info('Classify... ', informational),
      has_to_work((rdf_triple('http://ias.cs.tum.edu/kb/actionmodel.owl#forSituation', Frame, InputSituation); rdf_triple('http://ias.cs.tum.edu/kb/actionmodel.owl#forAction', Frame, InputAction)), unable_to_create_destination_situation),
      has_to_work(actionmodel:am_classify_action_situation(Model, InputAction, InputSituation, Action, Probability),
        unable_to_classify_situation),
      print(Probability),
      print_info(' done\n', informational),
      % am_attributes_nominal_or_numeric(Os, Ts),
      Value = Action),
  error(instantiation_error, _), fail),
  error(java_exception(JE), ExceptionRest),
  ( %prolog_current_frame(Frame),
    %prolog_frame_attribute(Frame, parent, ParentFrame),
    %prolog_frame_attribute(ParentFrame, goal, FrameGoal),print(FrameGoal),
    jpl_call(JE, printStackTrace, [], _), throw(error(java_exception(JE), ExceptionRest)))).
  
%% am_rdf_triple(actionmodel:'toSituation', ?Frame, ?Value).
%
% rdf_triple_hook implementation for am:toAction.
%
am_rdf_triple(Property, Frame, Value):-
  catch(catch(
    ( rdf_equal(Property, 'http://ias.cs.tum.edu/kb/actionmodel.owl#toSituation'),
      has_to_work(rdfs_instance_of(Frame, 'http://ias.cs.tum.edu/kb/actionmodel.owl#Prediction'), not_a_prediction_as_frame),
      % Get the actionModel
      has_to_work(rdf_triple('http://ias.cs.tum.edu/kb/actionmodel.owl#byActionmodel', Frame, ActionModel), no_byActionmodel),
    
      % Then calculate the action model
      has_to_work(actionmodel:am_get_action_model(ActionModel, Model), unable_to_create_action_model),
    
      print_info('Classify... ', informational),
      has_to_work((rdf_triple('http://ias.cs.tum.edu/kb/actionmodel.owl#forSituation', Frame, InputSituation); rdf_triple('http://ias.cs.tum.edu/kb/actionmodel.owl#forAction', Frame, InputAction)), unable_to_create_destination_situation),
      has_to_work(actionmodel:am_classify_situation_action(Model, InputAction, InputSituation, Situation, Probability),
        unable_to_classify_situation),
      print(Probability),
      print_info(' done\n', informational),
      % am_attributes_nominal_or_numeric(Os, Ts),
      Value = Situation),
    error(instantiation_error, _), fail),
    error(java_exception(JE), ExceptionRest), 
    ( %prolog_current_frame(Frame),
      %prolog_frame_attribute(Frame, parent, ParentFrame),
      %prolog_frame_attribute(ParentFrame, goal, FrameGoal),
      jpl_call(JE, printStackTrace, [], _), throw(error(java_exception(JE), ExceptionRest)))).

% To prevent the error that rdf_triple is already loaded from rdfs_computable.
% I use multifile, dynamic and discontiguous but the error still occurs when I use a
% standard definition. Why?!?
/*:- retract(user:(rdf_triple_hook(Property, Frame, Value):-
         actionmodel:am_rdf_triple(Property, Frame, Value))).*/
:- assert(user:(rdf_triple_hook(Property, Frame, Value):-
         actionmodel:am_rdf_triple(Property, Frame, Value))).
:- user:expand_term((rdf_triple_hook(Property, Frame, Value):-
    rdf_equal(Property, rdfs:range),
    rdfs_instance_of(Frame, am:'Feature'),
    rdf_has(Frame, am:'calculatedBy', RealProperty),
    rdf_triple(rdfs:range, RealProperty, Value)), X),
  assert(user:(X)).
:- user:expand_term((rdf_triple_hook(Property, Frame, Value):-
    rdf_equal(Property, rdfs:domain),
    rdfs_instance_of(Frame, am:'Feature'),
    rdf_has(Frame, am:'calculatedBy', RealProperty),
    rdf_triple(rdfs:domain, RealProperty, Value)), X),
  assert(user:(X)).
% rdf_triple(Property, Frame, Value):-
% 	rdf_equal(Property, 'http://ias.cs.tum.edu/kb/actionmodel.owl#toAction'),
% 	rdfs_instanceof(Frame, am:'ActionModel').
