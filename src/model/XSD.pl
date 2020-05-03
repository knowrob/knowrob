:- module(model_XSD,
    [ xsd_data_type(r),
      xsd_numeric_type(r),
      xsd_string_type(r),
      xsd_misc_type(r),
      xsd_data_basetype(r,?),
      xsd_data_type_name(r,?)
    ]).
/** <module> XML Schema Data Types.

@author Daniel Beßler
*/

%%
%
xsd_data_type(DataType) :-
  ( xsd_numeric_type(DataType);
    xsd_string_type(DataType);
    xsd_date_type(DataType);
    xsd_misc_type(DataType)
  ).

%%
%
xsd_data_basetype(DataType,number) :-
  xsd_numeric_type(DataType).
xsd_data_basetype(xsd:double,number).
xsd_data_basetype(xsd:float,number).
xsd_data_basetype(xsd:boolean,number).

xsd_data_basetype(DataType,string) :-
  xsd_string_type(DataType).
xsd_data_basetype(xsd:anyURI,number).

xsd_data_basetype(DataType,date) :-
  xsd_date_type(DataType).

xsd_data_basetype(xsd:base64Binary,blob).
xsd_data_basetype(xsd:hexBinary,blob).

%%
%
xsd_data_type_name(DataType,Name) :-
  ground(DataType),
  atom_concat('http://www.w3.org/2001/XMLSchema#',Name,DataType).

%%
% String data types are used for values that contains character strings.
%
xsd_string_type(xsd:'string').
xsd_string_type(xsd:'token').
xsd_string_type(xsd:'normalizedString').
xsd_string_type(xsd:'language').

%%
% Date and time data types are used for values that contain date and time.
%
xsd_date_type(xsd:'date').
xsd_date_type(xsd:'dateTime').
xsd_date_type(xsd:'duration').
xsd_date_type(xsd:'gDay').
xsd_date_type(xsd:'gMonth').
xsd_date_type(xsd:'gMonthDay').
xsd_date_type(xsd:'gYear').
xsd_date_type(xsd:'gYearMonth').
xsd_date_type(xsd:'time').

%%
% Decimal data types are used for numeric values.
%
xsd_numeric_type(xsd:'byte').
xsd_numeric_type(xsd:'decimal').
xsd_numeric_type(xsd:'int').
xsd_numeric_type(xsd:'integer').
xsd_numeric_type(xsd:'long').
xsd_numeric_type(xsd:'negativeInteger').
xsd_numeric_type(xsd:'nonNegativeInteger').
xsd_numeric_type(xsd:'nonPositiveInteger').
xsd_numeric_type(xsd:'positiveInteger').
xsd_numeric_type(xsd:'short').
xsd_numeric_type(xsd:'unsignedLong').
xsd_numeric_type(xsd:'unsignedInt').
xsd_numeric_type(xsd:'unsignedShort').
xsd_numeric_type(xsd:'unsignedByte').

%% 
%
xsd_misc_type(xsd:'anyURI').
xsd_misc_type(xsd:'base64Binary').
xsd_misc_type(xsd:'boolean').
xsd_misc_type(xsd:'double').
xsd_misc_type(xsd:'float').
xsd_misc_type(xsd:'hexBinary').
