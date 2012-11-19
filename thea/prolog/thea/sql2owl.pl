% Author: VV
% Date: Nov 2004 - April 2006
% 
% To do. Correct namespace handling in classes - individuals. Done 14/4
%        Quote literals/values. Done 14/4
%        Multiple columns in the ID of a class link
%	 Additional Where clause-filter in the populate_class
%	 Test if owl/rdf is correctly generated from individuals.


:- use_module('owl_parser.pl').

% -----------------------------------------------------------------

%	populate_class(+DBConnection, +Class)
%		
%	It constructs and executes an SQL query (against a
%	SWIs ODBC-package DBConnection), based on the class_link and any
%	property_links for this class. For each row, it creates one
%	Individual of class Class. Assigns Property-Value pairs to this
%	Individual as defined by the property_links having Class as
%	SubjectClass.
%       NOTE. All records of the Table linked to Class will
%	be populated. No filter is possible in this version. There is
%	always to workaround linking the class with a View/filter iso a
%	Table.

populate_class(DBConnection, Class) :-
	process_class_link(Class,CTW),!,
        ctw_to_sql(CTW,SQL_Query),
	execute_sql(DBConnection, Class,SQL_Query).	

%	populate_property(+DBConnection, +Property)
%		
%	It constructs and executes an SQL query (against a
%	SWIs ODBC-package DBConnection), based on the property_link 
%	for the Property. It creates one Individual of class as defined
%	in the Subject property_link for each row returned. Assigns
%	a Property-Value pair to this Individual as defined by the
%	property_link. 
%	NOTE: Use only for properties having link with a 'Class' Subject 

populate_property(DBConnection, Property) :-
	property_link(Class, Property, ClassOrObject, Options), 
	process_property_link(Class, Property,ClassOrObject, Options, CTW),
        ctw_to_sql(CTW,SQL_Query), 
	execute_sql(DBConnection, Class,SQL_Query).	

%	process_class_link(+Class, -CTW)
%
%	Processes all property_links of a Class and returns a CTW term,
%	containing the SELECT, FROM and WHERE elements of an SQL query,
%	to be used for populating Individuals with this class link. 

process_class_link(Class,CTW) :-
        assert(property_link(Class,'_IID',Class,[])),
	findall(CTW1,
		(   property_link(Class, PropertyName, ClassOrObject, Options), 
		    process_property_link(Class, PropertyName,ClassOrObject, Options, CTW1)),
		CTW_List),		
	merge_ctw(CTW_List, [CTW]),
        retract(property_link(Class,'_IID',Class,[])).


%	execute_sql(+DBConnection, +Class, +SQL_Query)
%		
%	Executes SQL_Query against SWI-Prolog's ODBC package
%	DBConnection. 
%	For each row returned, it asserts an 'Individual' fact.

execute_sql(DBConnection, Class,SQL_Query) :-
  odbc_query(DBConnection, SQL_Query, X, [source(true)]),
  X =.. [row|L], 
  make_individual_from_row(L,IID,I),
  assert(individual(IID,[],[Class],I)),
  fail.

execute_sql(_, _,_).

%	make_individual_from_row(+ColumnList,-ID,-PVList)
%		
%	Converts a column(_,Column,Value) list (the results of the
%	odbc_query) into a Property-Value pair list PVList.
%	It treats the Value of the Column named _IID as 
%	the individual's identifier (ID) and not as a property value.

make_individual_from_row([],_,[]) :- !.

make_individual_from_row([column(_,'_IID',ID)|Rest],ID,Rest1) :-
	make_individual_from_row(Rest,ID,Rest1),!.

make_individual_from_row([column(_,Column,Value)|Rest],ID,[value(Column, Value)|Rest1]) :-
	make_individual_from_row(Rest,ID,Rest1).

% -----------------------------------------------------------------------
%	process_property_link(+ClassOrSubject, +PropertyName, +ClassOrObject, +Options, -CTW)
%
%	Processes the elenents of a property_link (Subject,
%	Property, Objectm Options) and returns a CTW term,
%	containing the SELECT, FROM and WHERE elements of an SQL query,
%	to be used for populating Individuals with this property link.

process_property_link(ClassOrSubject, PropertyName, ClassOrObject,Options, CTW) :-
	(   class_link(ClassOrSubject, SubjectTable, SubjectColumn),
	    concat_atom([SubjectTable,'.',SubjectColumn],Subject1),
	    concat_atom(['concat(''',ClassOrSubject,''',''-'',',Subject1,')'],Subject)
	    ;
	    Subject = ClassOrSubject, atom_concat(Ts,_,Subject), atom_concat(SubjectTable,'.',Ts) 	),
	(   class_link(ClassOrObject, ObjectTable, ObjectColumn),
	    concat_atom([ObjectTable,'.',ObjectColumn],Object1),
	    concat_atom(['concat(''',ClassOrObject,''',''-'',',Object1,')'],Object)
	    ;
            % Object is not a class, 
	    Object = ClassOrObject,
            (	atom_concat(To,_,Object), atom_concat(ObjectTable,'.',To) ;
          	% !!! if Table prefix not in object use Subjects
                ObjectTable = SubjectTable)
	),
	% Here process the Options
        (   member(tpf(TPF_List),Options), 
	    (   member(sf(SF),Options) ; SF = Subject1),
	    (   member(op(OP),Options) ; OP = Object1 ),
	    build_where_list(TPF_List,SF,OP,Where_SQL),!
	    ;  
	    (	member(sf(SF),Options), (member(op(OP),Options) ; OP = Object1)
		; 
		member(op(OP),Options), SF = Subject1
            ), !, 
	    Where_SQL = [SF-OP], TPF_List = []
            ; 
            Where_SQL = [], TPF_List = []
	),!,
        build_from_list(SubjectTable, ObjectTable, TPF_List,[], From_SQL),
        Column_SQL = [Subject-'_IID', Object-PropertyName],
	CTW = ctw(Column_SQL,From_SQL,Where_SQL).


%	process_where_list/4, process_where_list/3, build_from_list/5
%
%	Utility predicates building the FROM and WHERE elements of a
%	CTW term. Called by process property link.

build_where_list([],SF,OP, [SF-OP]) :- !.
build_where_list([PK-FK|TPF_List],SF,OP, [SF-PK|List]) :-   
	build_where_list([PK-FK|TPF_List],OP,List).

build_where_list([_-FK],OP,[FK-OP]) :- !.
build_where_list([_-FK,PK1-FK1|T],OP, [FK-PK1 | Out]) :-
	build_where_list([PK1-FK1|T],OP,Out).

build_from_list(SubjectTable, ObjectTable, [PK-_|T], In, [Table| Out]) :-
       atom_concat(PK1,_,PK), atom_concat(Table,'.',PK1),
       build_from_list(SubjectTable, ObjectTable, T, In, Out).	
build_from_list(SubjectTable, ObjectTable, [], List, [SubjectTable, ObjectTable | List]) :- !.


% -----------------------------------------------------------------

%	ctw_to_sql(+CTW,-SQL_String)

%	Converts a CTW term containing the SELECT, FROM and WHERE
%	elements of an SQL query, to an SQL Query string ready for
%	execution by ODBC package. 

ctw_to_sql(ctw(C1,T1,W1),SQL_String) :-
        list_to_set(C1,C), list_to_set(T1,T), list_to_set(W1,W),
	expand_sql_list(C,Cexp,' as ', ' , '), append(['Select '],Cexp,Tmp1),
	append(Tmp1, [' From '],Tmp2), 
	expand_sql_list(T,Texp), append(Tmp2, Texp, Tmp3),
        (   W = [], Tmp5 = Tmp3  
	    ;
	    append(Tmp3,[' Where '], Tmp4),
	    expand_sql_list(W,Wexp, ' = ', ' and '),append(Tmp4, Wexp, Tmp5)
	),
	list_sql(Tmp5,SQL_String).

%	expand_sql_list(+List,-CSList).
%		
%	Converts a List into a comma separated list. Used by
%	build_sql_class to create the list of tables in the FROM
%	clause of the query

expand_sql_list([],[]).
expand_sql_list([Table],[Table]).
expand_sql_list([Table|Rest],[Table, ',' | Rest1]):-
	expand_sql_list(Rest,Rest1).

%	expand_sql_list(+XYList,-ListCS, +Operator, +Separator).
%		
%	Converts a List with X-Y elements into a list with elements
%	X Operator X separated by Separator. Used by build_sql_class to
%	create the SELECT and the WHERE clauses of the query

expand_sql_list([],[],_,_) :- !.
expand_sql_list([X-Y],[X , Operator, Y],Operator,_) :- !.
expand_sql_list([X-Y|Rest],[X, Operator, Y, Connector | Rest1],Operator,Connector) :-
	expand_sql_list(Rest,Rest1,Operator,Connector).


%	merge_ctw(+CTW_List, -Merged_CTW_List)
%		
%	The CTW_list is a list with ctw(C,T,W) elements. 
%	The result is a one-element ctw(Cm,Tm,Wm) list where Cm,Tm
%	and Wm are the merged lists of all C, T and W respectively.
%	Used by build_class_sql to merge the SELECT, FROM and WHERE
%	clauses of the individual property_link SQLs.
	
merge_ctw([],[]) :-!.
merge_ctw([X],[X]) :- !.

merge_ctw([H1,H2|T1],T2) :-
	merge_ctw(H1,H2,H3),
	merge_ctw([H3|T1],T2).

merge_ctw(ctw(C1,T1,W1),ctw(C2,T2,W2),ctw(C3,T3,W3)) :-
	merge(C1,C2,C3),
	merge(T1,T2,T3),
	merge(W1,W2,W3).


%	list_sql(+List, -String).
%
%       Utility predicate to concat a list to its string representation 

list_sql([H|T],Sql) :- 
   string_to_atom(Sql1, H),
   list_sql(T,Sql2),
   string_concat(Sql1,Sql2,Sql).

list_sql([],'').















