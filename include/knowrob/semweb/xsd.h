/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_XSD_H
#define KNOWROB_XSD_H

#include <string>
#include "knowrob/terms/IRIAtom.h"

namespace knowrob::xsd {
	constexpr std::string_view prefix = "http://www.w3.org/2001/XMLSchema#";

	const IRIAtomPtr IRI_anyURI = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#anyURI");
	const IRIAtomPtr IRI_string = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#string");
	const IRIAtomPtr IRI_token = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#token");
	const IRIAtomPtr IRI_normalizedString = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#normalizedString");
	const IRIAtomPtr IRI_language = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#language");

	const IRIAtomPtr IRI_int = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#int");
	const IRIAtomPtr IRI_integer = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#integer");
	const IRIAtomPtr IRI_positiveInteger = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#positiveInteger");
	const IRIAtomPtr IRI_negativeInteger = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#negativeInteger");
	const IRIAtomPtr IRI_nonNegativeInteger = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#nonNegativeInteger");
	const IRIAtomPtr IRI_nonPositiveInteger = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#nonPositiveInteger");
	const IRIAtomPtr IRI_unsignedInt = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#unsignedInt");
	const IRIAtomPtr IRI_long = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#long");
	const IRIAtomPtr IRI_unsignedLong = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#unsignedLong");
	const IRIAtomPtr IRI_short = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#short");
	const IRIAtomPtr IRI_unsignedShort = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#unsignedShort");
	const IRIAtomPtr IRI_byte = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#byte");
	const IRIAtomPtr IRI_unsignedByte = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#unsignedByte");
	const IRIAtomPtr IRI_boolean = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#boolean");

	const IRIAtomPtr IRI_decimal = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#decimal");
	const IRIAtomPtr IRI_double = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#double");
	const IRIAtomPtr IRI_float = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#float");

	const IRIAtomPtr IRI_date = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#date");
	const IRIAtomPtr IRI_dateTime = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#dateTime");
	const IRIAtomPtr IRI_duration = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#duration");
	const IRIAtomPtr IRI_gDay = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#gDay");
	const IRIAtomPtr IRI_gMonth = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#gMonth");
	const IRIAtomPtr IRI_gMonthDay = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#gMonthDay");
	const IRIAtomPtr IRI_gYear = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#gYear");
	const IRIAtomPtr IRI_gYearMonth = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#gYearMonth");
	const IRIAtomPtr IRI_time = IRIAtom::Tabled("http://www.w3.org/2001/XMLSchema#time");

	bool isNumericType(std::string_view iri);

	bool isDoubleType(std::string_view iri);

	bool isIntegerType(std::string_view iri);

	bool isStringType(std::string_view iri);

	bool isBooleanType(std::string_view iri);

	bool isDateType(std::string_view iri);

} // knowrob::xsd

#endif //KNOWROB_XSD_H
