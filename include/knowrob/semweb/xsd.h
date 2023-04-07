//
// Created by daniel on 03.04.23.
//

#ifndef KNOWROB_XSD_H
#define KNOWROB_XSD_H

#include <string>

namespace knowrob::xsd {
    const std::string IRI_anyURI = "http://www.w3.org/2001/XMLSchema#anyURI";
    const std::string IRI_string = "http://www.w3.org/2001/XMLSchema#string";
    const std::string IRI_token = "http://www.w3.org/2001/XMLSchema#token";
    const std::string IRI_normalizedString = "http://www.w3.org/2001/XMLSchema#normalizedString";
    const std::string IRI_language = "http://www.w3.org/2001/XMLSchema#language";

    const std::string IRI_int = "http://www.w3.org/2001/XMLSchema#int";
    const std::string IRI_integer = "http://www.w3.org/2001/XMLSchema#integer";
    const std::string IRI_positiveInteger = "http://www.w3.org/2001/XMLSchema#positiveInteger";
    const std::string IRI_negativeInteger = "http://www.w3.org/2001/XMLSchema#negativeInteger";
    const std::string IRI_nonNegativeInteger = "http://www.w3.org/2001/XMLSchema#nonNegativeInteger";
    const std::string IRI_nonPositiveInteger = "http://www.w3.org/2001/XMLSchema#nonPositiveInteger";
    const std::string IRI_unsignedInt = "http://www.w3.org/2001/XMLSchema#unsignedInt";
    const std::string IRI_long = "http://www.w3.org/2001/XMLSchema#long";
    const std::string IRI_unsignedLong = "http://www.w3.org/2001/XMLSchema#unsignedLong";
    const std::string IRI_short = "http://www.w3.org/2001/XMLSchema#short";
    const std::string IRI_unsignedShort = "http://www.w3.org/2001/XMLSchema#unsignedShort";
    const std::string IRI_byte = "http://www.w3.org/2001/XMLSchema#byte";
    const std::string IRI_unsignedByte = "http://www.w3.org/2001/XMLSchema#unsignedByte";

    const std::string IRI_decimal = "http://www.w3.org/2001/XMLSchema#decimal";
    const std::string IRI_double = "http://www.w3.org/2001/XMLSchema#double";
    const std::string IRI_float = "http://www.w3.org/2001/XMLSchema#float";

    const std::string IRI_date = "http://www.w3.org/2001/XMLSchema#date";
    const std::string IRI_dateTime = "http://www.w3.org/2001/XMLSchema#dateTime";
    const std::string IRI_duration = "http://www.w3.org/2001/XMLSchema#duration";
    const std::string IRI_gDay = "http://www.w3.org/2001/XMLSchema#gDay";
    const std::string IRI_gMonth = "http://www.w3.org/2001/XMLSchema#gMonth";
    const std::string IRI_gMonthDay = "http://www.w3.org/2001/XMLSchema#gMonthDay";
    const std::string IRI_gYear = "http://www.w3.org/2001/XMLSchema#gYear";
    const std::string IRI_gYearMonth = "http://www.w3.org/2001/XMLSchema#gYearMonth";
    const std::string IRI_time = "http://www.w3.org/2001/XMLSchema#time";

    const std::string IRI_boolean = "http://www.w3.org/2001/XMLSchema#boolean";

    bool isNumericType(std::string_view iri);

    bool isDoubleType(std::string_view iri);

    bool isIntegerType(std::string_view iri);

    bool isStringType(std::string_view iri);

    bool isBooleanType(std::string_view iri);

    bool isDateType(std::string_view iri);

} // knowrob::xsd

#endif //KNOWROB_XSD_H
