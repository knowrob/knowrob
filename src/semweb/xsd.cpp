/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

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
        { return iri == IRI_boolean->stringForm(); }

    protected:
        XSD() {
            stringTypes_.insert(IRI_anyURI->stringForm());
            stringTypes_.insert(IRI_string->stringForm());
            stringTypes_.insert(IRI_token->stringForm());
            stringTypes_.insert(IRI_normalizedString->stringForm());
            stringTypes_.insert(IRI_language->stringForm());

            integerTypes_.insert(IRI_byte->stringForm());
            integerTypes_.insert(IRI_int->stringForm());
            integerTypes_.insert(IRI_integer->stringForm());
            integerTypes_.insert(IRI_long->stringForm());
            integerTypes_.insert(IRI_negativeInteger->stringForm());
            integerTypes_.insert(IRI_nonNegativeInteger->stringForm());
            integerTypes_.insert(IRI_nonPositiveInteger->stringForm());
            integerTypes_.insert(IRI_positiveInteger->stringForm());
            integerTypes_.insert(IRI_short->stringForm());
            integerTypes_.insert(IRI_unsignedLong->stringForm());
            integerTypes_.insert(IRI_unsignedInt->stringForm());
            integerTypes_.insert(IRI_unsignedShort->stringForm());
            integerTypes_.insert(IRI_unsignedByte->stringForm());

            doubleTypes_.insert(IRI_decimal->stringForm());
            doubleTypes_.insert(IRI_double->stringForm());
            doubleTypes_.insert(IRI_float->stringForm());

            dateTypes_.insert(IRI_date->stringForm());
            dateTypes_.insert(IRI_dateTime->stringForm());
            dateTypes_.insert(IRI_duration->stringForm());
            dateTypes_.insert(IRI_gDay->stringForm());
            dateTypes_.insert(IRI_gMonth->stringForm());
            dateTypes_.insert(IRI_gMonthDay->stringForm());
            dateTypes_.insert(IRI_gYear->stringForm());
            dateTypes_.insert(IRI_gYearMonth->stringForm());
            dateTypes_.insert(IRI_time->stringForm());
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
