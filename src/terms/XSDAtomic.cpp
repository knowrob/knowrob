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
				{xsd::IRI_string->stringForm(),             XSDType::STRING},
				{xsd::IRI_anyURI->stringForm(),             XSDType::STRING},
				{xsd::IRI_boolean->stringForm(),            XSDType::BOOLEAN},
				{xsd::IRI_double->stringForm(),             XSDType::DOUBLE},
				{xsd::IRI_float->stringForm(),              XSDType::FLOAT},
				{xsd::IRI_integer->stringForm(),            XSDType::INTEGER},
				{xsd::IRI_int->stringForm(),                XSDType::INTEGER},
				{xsd::IRI_nonNegativeInteger->stringForm(), XSDType::NON_NEGATIVE_INTEGER},
				{xsd::IRI_long->stringForm(),               XSDType::LONG},
				{xsd::IRI_short->stringForm(),              XSDType::SHORT},
				{xsd::IRI_unsignedLong->stringForm(),       XSDType::UNSIGNED_LONG},
				{xsd::IRI_unsignedInt->stringForm(),        XSDType::UNSIGNED_INT},
				{xsd::IRI_unsignedShort->stringForm(),      XSDType::UNSIGNED_SHORT}
		};
		auto it = typeIRIs.find(iri);
		if (it != typeIRIs.end()) return it->second;
		KB_WARN("Unknown XSD type IRI {} treated as string.", iri);
		return XSDType::STRING;
	}

	std::string_view xsdTypeToIRI(XSDType type) {
		static std::map<XSDType, std::string_view> typeIRIs = {
				{XSDType::STRING,               xsd::IRI_string->stringForm()},
				{XSDType::BOOLEAN,              xsd::IRI_boolean->stringForm()},
				{XSDType::DOUBLE,               xsd::IRI_double->stringForm()},
				{XSDType::FLOAT,                xsd::IRI_float->stringForm()},
				{XSDType::INTEGER,              xsd::IRI_integer->stringForm()},
				{XSDType::NON_NEGATIVE_INTEGER, xsd::IRI_nonNegativeInteger->stringForm()},
				{XSDType::LONG,                 xsd::IRI_long->stringForm()},
				{XSDType::SHORT,                xsd::IRI_short->stringForm()},
				{XSDType::UNSIGNED_LONG,        xsd::IRI_unsignedLong->stringForm()},
				{XSDType::UNSIGNED_INT,         xsd::IRI_unsignedInt->stringForm()},
				{XSDType::UNSIGNED_SHORT,       xsd::IRI_unsignedShort->stringForm()}
		};
		auto it = typeIRIs.find(type);
		if (it != typeIRIs.end()) return it->second;
		KB_WARN("Unknown XSD type {} treated as string.", static_cast<int>(type));
		return xsd::IRI_string->stringForm();
	}

	std::string_view XSDAtomic::xsdTypeIRI() const {
		return xsdTypeToIRI(xsdType());
	}
}
