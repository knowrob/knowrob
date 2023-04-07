//
// Created by daniel on 03.04.23.
//

#include <set>
#include "knowrob/semweb/xsd.h"

namespace knowrob::xsd {
    class XSD {
    public:
        static XSD& get() {
            static XSD instance;
            return instance;
        }

        static bool isStringType(std::string_view &iri)
        { return get().stringTypes_.count(iri)>0; }

        static bool isIntegerType(std::string_view &iri)
        { return get().integerTypes_.count(iri)>0; }

        static bool isDoubleType(std::string_view &iri)
        { return get().doubleTypes_.count(iri)>0; }

        static bool isDateType(std::string_view &iri)
        { return get().dateTypes_.count(iri)>0; }

        static bool isBooleanType(std::string_view &iri)
        { return iri == IRI_boolean; }

    protected:
        XSD() {
            stringTypes_.insert(IRI_anyURI);
            stringTypes_.insert(IRI_string);
            stringTypes_.insert(IRI_token);
            stringTypes_.insert(IRI_normalizedString);
            stringTypes_.insert(IRI_language);

            integerTypes_.insert(IRI_byte);
            integerTypes_.insert(IRI_int);
            integerTypes_.insert(IRI_integer);
            integerTypes_.insert(IRI_long);
            integerTypes_.insert(IRI_negativeInteger);
            integerTypes_.insert(IRI_nonNegativeInteger);
            integerTypes_.insert(IRI_nonPositiveInteger);
            integerTypes_.insert(IRI_positiveInteger);
            integerTypes_.insert(IRI_short);
            integerTypes_.insert(IRI_unsignedLong);
            integerTypes_.insert(IRI_unsignedInt);
            integerTypes_.insert(IRI_unsignedShort);
            integerTypes_.insert(IRI_unsignedByte);

            doubleTypes_.insert(IRI_decimal);
            doubleTypes_.insert(IRI_double);
            doubleTypes_.insert(IRI_float);

            dateTypes_.insert(IRI_date);
            dateTypes_.insert(IRI_dateTime);
            dateTypes_.insert(IRI_duration);
            dateTypes_.insert(IRI_gDay);
            dateTypes_.insert(IRI_gMonth);
            dateTypes_.insert(IRI_gMonthDay);
            dateTypes_.insert(IRI_gYear);
            dateTypes_.insert(IRI_gYearMonth);
            dateTypes_.insert(IRI_time);
        }
        std::set<std::string_view> stringTypes_;
        std::set<std::string_view> integerTypes_;
        std::set<std::string_view> doubleTypes_;
        std::set<std::string_view> dateTypes_;
    };

    bool isStringType(std::string_view iri)
    {
        return XSD::isStringType(iri);
    }

    bool isNumericType(std::string_view iri)
    {
        return isDoubleType(iri) || isIntegerType(iri);
    }

    bool isIntegerType(std::string_view iri)
    {
        return XSD::isIntegerType(iri);
    }

    bool isDoubleType(std::string_view iri)
    {
        return XSD::isDoubleType(iri);
    }

    bool isBooleanType(std::string_view iri)
    {
        return XSD::isBooleanType(iri);
    }

    bool isDateType(std::string_view iri)
    {
        return XSD::isDateType(iri);
    }
}
