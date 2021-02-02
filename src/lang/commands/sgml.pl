:- module(lang_sgml, []).

:- use_module(library('semweb/rdf_db'),
		[ rdf_global_term/2 ]).
:- use_module(library('lang/db'),
		[ get_unique_name/2 ]).
:- use_module(library('lang/compiler')).

%% query commands
:- query_compiler:add_command(iri_xml_namespace, [ask]).
:- query_compiler:add_command(new_iri,           [ask,tell]).

%%
% tell queries can use new_iri/1 and new_iri/2 to generate
% IRI's that have not been used so far.
%
% FIXME: it could happen that if in one compilation multiple new_iri's
%         are generated that both have the same IRI. very unlikely, but still...
%
query_compiler:step_expand(new_iri(IRI),
		pragma(get_unique_name(Type,IRI)), _) :-
	rdf_global_term(rdf:'Resource', Type).

query_compiler:step_expand(new_iri(IRI,Type),
		pragma(get_unique_name(Type,IRI)), _).


%% query variables
query_compiler:step_var(
		iri_xml_namespace(IRI,Namespace,Localname),
		Ctx, Var) :-
	query_compiler:get_var([IRI,Namespace,Localname],Ctx,Var).

%% query compilation
query_compiler:step_compile(
		iri_xml_namespace(IRI,NS,Name),
		Ctx, Pipeline) :-
	query_compiler:var_key_or_val(IRI, Ctx, IRI0),
	query_compiler:var_key_or_val(NS, Ctx, NS0),
	query_compiler:var_key_or_val(Name, Ctx, Name0),
	findall(Step,
		% first extract the name from the IRI and set new field "t_name".
		% here we use the remainder after the last '#' as name.
		% FIXME: this is not entirely accurate, see the documentation of iri_xml_namespace:
		%			https://www.swi-prolog.org/pldoc/man?predicate=iri_xml_namespace/3
		(	Step=['$set', ['t_name', ['$last', ['$split', array([IRI0, string('#')])]]]]
		% next set field "t_ns" to remaining prefix of IRI
		;	Step=['$set', ['t_ns',   ['$substr', array([IRI0, int(0),
				['$subtract', array([
					['$strLenCP',IRI0],
					['$strLenCP',string('$t_name')]
				])]
			])]]]
		% assign arguments if needed using fields created above
		;	query_compiler:set_if_var(NS,   string('$t_ns'),   Ctx, Step)
		;	query_compiler:set_if_var(Name, string('$t_name'), Ctx, Step)
		% finally match using concat operator
		;	query_compiler:match_equals(IRI0, ['$concat', array([NS0,Name0])], Step)
		% cleanup
		;	Step=['$unset', array([string('t_ns'),string('t_name')])]
		),
		Pipeline).


		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- begin_tests('lang_sgml').

test('new_iri(-IRI)'):-
	lang_query:test_command(
		(new_iri(IRI), atom_concat(IRI,foo,X)),
		X, string(foo)),
	assert_true(atom(X)),
	assert_true(atom_concat(_,foo,X)).

test('iri_xml_namespace(+IRI,-NS,-Name)'):-
	rdf_global_term(rdf:'Resource', Resource),
	lang_query:test_command(
		iri_xml_namespace(X,NS,Name),
		X, string(Resource)),
	assert_true(atom(NS)),
	assert_equals(Name,'Resource'),
	assert_true(atomic_concat(NS,Name,Resource)).

:- end_tests('lang_sgml').
