
:- module(knowrob_xsd,
    [
      xsd_number_type/1,
      xsd_string_type/1,
      xsd_boolean_type/1
    ]).
/** <module> Prolog XSD interface.

@author Daniel Beßler
*/

xsd_number_type('http://www.w3.org/2001/XMLSchema#float').
xsd_number_type('http://www.w3.org/2001/XMLSchema#double').
xsd_number_type('http://www.w3.org/2001/XMLSchema#byte').
xsd_number_type('http://www.w3.org/2001/XMLSchema#short').
xsd_number_type('http://www.w3.org/2001/XMLSchema#int').
xsd_number_type('http://www.w3.org/2001/XMLSchema#long').
xsd_number_type('http://www.w3.org/2001/XMLSchema#unsignedByte').
xsd_number_type('http://www.w3.org/2001/XMLSchema#unsignedShort').
xsd_number_type('http://www.w3.org/2001/XMLSchema#unsignedInt').
xsd_number_type('http://www.w3.org/2001/XMLSchema#unsignedLong').

xsd_string_type('http://www.w3.org/2001/XMLSchema#string').

xsd_boolean_type('http://www.w3.org/2001/XMLSchema#boolean').
