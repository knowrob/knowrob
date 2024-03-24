:- module(mongolog_sgml, []).
/** <module> XML namespaces in mongolog programs.

The following predicates are supported:

| Predicate            | Arguments |
| ---                  | ---       |
| iri_xml_namespace/3  | +IRI, -Namespace, -Localname |

In addition, a command new_iri/1 can be used to generate a unique
name that is not used yet.
A variant, new_iri/2, allows to provide a IRI prefix
for the new IRI.

@author Daniel Beßler
@see https://www.swi-prolog.org/pldoc/man?section=xmlns
@license BSD
*/

:- use_module(library('semweb/rdf_db'),
		[ rdf_global_term/2 ]).
:- use_module(library('mongolog/triple'),
		[ get_unique_name/2 ]).
:- use_module('mongolog').

%% query commands
:- mongolog:add_command(iri_xml_namespace).
:- mongolog:add_command(new_iri).

%%
% projection queries can use new_iri/1 and new_iri/2 to generate
% IRI's that have not been used so far.
%
mongolog:step_expand(
		new_iri(IRI),
		pragma(get_unique_name(Type,IRI))) :-
	rdf_global_term(rdf:'Resource', Type).

mongolog:step_expand(
		new_iri(IRI,Type),
		pragma(get_unique_name(Type,IRI))).

%% query compilation
mongolog:step_compile(
		iri_xml_namespace(IRI,NS,Name),
		Ctx, Pipeline) :-
	mongolog:var_key_or_val(IRI, Ctx, IRI0),
	mongolog:var_key_or_val(NS, Ctx, NS0),
	mongolog:var_key_or_val(Name, Ctx, Name0),
	findall(Step,
		% first extract the name from the IRI and set new field "t_name".
		% here we use the remainder after the last '#' as name.
		% Note: this is not entirely accurate, see the documentation of iri_xml_namespace:
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
		;	mongolog:set_if_var(NS,   string('$t_ns'),   Ctx, Step)
		;	mongolog:set_if_var(Name, string('$t_name'), Ctx, Step)
		% finally match using concat operator
		;	mongolog:match_equals(IRI0, ['$concat', array([NS0,Name0])], Step)
		% cleanup
		;	Step=['$unset', array([string('t_ns'),string('t_name')])]
		),
		Pipeline).


		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- begin_tests('mongolog_sgml').

test('new_iri(-IRI)'):-
	mongolog:test_call(
		(new_iri(IRI), atom_concat(IRI,Suffix,X)), Suffix, foo),
	assert_true(atom(X)),
	assert_true(atom_concat(_,foo,X)).

test('iri_xml_namespace(+IRI,-NS,-Name)'):-
	rdf_global_term(rdf:'Resource', Resource),
	mongolog:test_call(
		iri_xml_namespace(X,NS,Name),
		X, Resource),
	assert_true(atom(NS)),
	assert_equals(Name,'Resource'),
	assert_true(atomic_concat(NS,Name,Resource)).

:- end_tests('mongolog_sgml').
