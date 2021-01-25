:- module(lang_sgml, []).

:- use_module(library('lang/compiler')).
:- use_module(library('lang/db')
		[ get_unique_name/2 ]).

%% query commands
:- query_compiler:add_command(iri_xml_namespace, [ask]).
:- query_compiler:add_command(new_iri,           [tell]).


%%
% tell queries can use new_iri/1 and new_iri/2 to generate
% IRI's that have not been used so far.
%
% FIXME: it could happen that if in one compilation multiple new_iri's
%         are generated that both have the same IRI. very unlikely, but still...
%
query_compiler:step_expand(new_iri(IRI),
		pragma(get_unique_name(dul:'Entity',IRI)), _).

query_compiler:step_expand(new_iri(IRI,Type),
		pragma(get_unique_name(Type,IRI)), _).


%% query variables
query_compiler:step_var(
		iri_xml_namespace(IRI,Namespace,Localname),
		Var) :-
	query_compiler:get_var([IRI,Namespace,Localname],Var).

%% query compilation
query_compiler:step_compile(
		iri_xml_namespace(IRI,NS,Name), _,
		Pipeline) :-
	query_compiler:var_key_or_val(IRI,IRI0),
	query_compiler:var_key_or_val(NS,NS0),
	query_compiler:var_key_or_val(Name,Name0),
	findall(Step,
		% first extract the name from the IRI and set new field "t_name".
		% here we use the remainder after the last '#' as name.
		% FIXME: this is not entirely accurate, see the documentation of iri_xml_namespace:
		%			https://www.swi-prolog.org/pldoc/man?predicate=iri_xml_namespace/3
		(	['$set', ['t_name', ['$last', ['$split', array([IRI0, string('#')])]]]]
		% next set field "t_ns" to remaining prefix of IRI
		;	['$set', ['t_ns',   ['$substr', array([IRI0, int(0),
				['$subtract', array([
					['$strLenCP',IRI0],
					['$strLenCP',string('$t_name')]
				])]
			])]]]
		% unify arguments if needed using fields created above
		;	set_if_var(NS,   string('$t_ns'),   Step)
		;	set_if_var(Name, string('$t_name'), Step)
		% finally match using concat operator
		;	match_equals(IRI0, ['$concat', array([NS0,Name0])], Step)
		% cleanup
		;	['$unset', array([string('t_ns'),string('t_name')])]
		),
		Pipeline).
