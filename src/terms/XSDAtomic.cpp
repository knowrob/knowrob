/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "map"
#include "knowrob/terms/XSDAtomic.h"
#include "knowrob/semweb/xsd.h"
#include "knowrob/Logger.h"

namespace knowrob {
	XSDType xsdTypeFromIRI(std::string_view iri) {
		static std::map<std::string_view, XSDType> typeIRIs = {
				{xsd::IRI_string,             XSDType::STRING},
				{xsd::IRI_anyURI,             XSDType::STRING},
				{xsd::IRI_boolean,            XSDType::BOOLEAN},
				{xsd::IRI_double,             XSDType::DOUBLE},
				{xsd::IRI_float,              XSDType::FLOAT},
				{xsd::IRI_integer,            XSDType::INTEGER},
				{xsd::IRI_int,                XSDType::INTEGER},
				{xsd::IRI_nonNegativeInteger, XSDType::NON_NEGATIVE_INTEGER},
				{xsd::IRI_long,               XSDType::LONG},
				{xsd::IRI_short,              XSDType::SHORT},
				{xsd::IRI_unsignedLong,       XSDType::UNSIGNED_LONG},
				{xsd::IRI_unsignedInt,        XSDType::UNSIGNED_INT},
				{xsd::IRI_unsignedShort,      XSDType::UNSIGNED_SHORT}
		};
		auto it = typeIRIs.find(iri);
		if (it != typeIRIs.end()) return it->second;
		KB_WARN("Unknown XSD type IRI {} treated as string.", iri);
		return XSDType::STRING;
	}

	std::string_view xsdTypeToIRI(XSDType type) {
		static std::map<XSDType, std::string_view> typeIRIs = {
				{XSDType::STRING,               xsd::IRI_string},
				{XSDType::BOOLEAN,              xsd::IRI_boolean},
				{XSDType::DOUBLE,               xsd::IRI_double},
				{XSDType::FLOAT,                xsd::IRI_float},
				{XSDType::INTEGER,              xsd::IRI_integer},
				{XSDType::NON_NEGATIVE_INTEGER, xsd::IRI_nonNegativeInteger},
				{XSDType::LONG,                 xsd::IRI_long},
				{XSDType::SHORT,                xsd::IRI_short},
				{XSDType::UNSIGNED_LONG,        xsd::IRI_unsignedLong},
				{XSDType::UNSIGNED_INT,         xsd::IRI_unsignedInt},
				{XSDType::UNSIGNED_SHORT,       xsd::IRI_unsignedShort}
		};
		auto it = typeIRIs.find(type);
		if (it != typeIRIs.end()) return it->second;
		KB_WARN("Unknown XSD type {} treated as string.", static_cast<int>(type));
		return xsd::IRI_string;
	}

	std::string_view XSDAtomic::xsdTypeIRI() const {
		return xsdTypeToIRI(xsdType());
	}
}
