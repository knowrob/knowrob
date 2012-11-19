
% :- use_module('sql2owl.pl').
:- dynamic class_link/3.
:- dynamic property_link/4.

:-consult('sql2owl.pl').

% ----------------------------------------------------------------------
%       sql2owl_connect(+Usr,+Pwd) 
%	
%	Makes a persistent connection ('mysql') to an ODBC source using
%	SWI-Prolog's ODBC package.

sql2owl_connect(Usr,Pwd) :- 
	odbc_connect('mysql', _,
	       	 [ user(Usr),
                   password(Pwd),
                   alias(mysql),
                   open(once)
                 ]).

% ----------------------------------------------------------------------
%       class_link(+Class, +Table, +ID_column)
%	
%	Dynamic predicate to define a link between a database
%	Table and the derived Class in the owl ontology.
%	The ID_is usually the primary key of the
%	table and is used to identify the resulting individual.
%	NOTE: Only single ID column is supported currently.

class_link('Person', 'swc_researchers', rid).
class_link('Organisation', 'swc_organisations', oid).
class_link('http://owl.org/swc_ontology#Project','swc_projects',pid).


%	property_link(+ClassOrSubject, +Property, +ClassOrObject,
%	Options)
%	
%	Dynamic predicate to define a link between a database
%	relation and the property-value pairs of individuals.
%	
%	Links an individual of Subject Class to one of Object Class 
%	with Property. It assigns the value (literal or Individual
%	URL) Object to the Property of each individual of the
%	Subject Class. 

property_link('Person', 'works_for', 'swc_organisations.title',
      [op('swc_organisations.oid'),
       tpf(['swc_researchers_organisations.rid'-'swc_researchers_organisations.oid'])]).
	      
property_link('Person', 'works_for_2', 'Organisation',
               [tpf(['swc_researchers_organisations.rid'-'swc_researchers_organisations.oid'])]).

property_link('Person','''example:Name''','swc_researchers.name',[]).

property_link('swc_researchers.rid','Name','name',[]).
property_link('Person','Name2','name',[]).


