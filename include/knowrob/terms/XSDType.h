/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_XSD_TYPE_H
#define KNOWROB_XSD_TYPE_H

#include "string_view"

namespace knowrob {
	/**
	 * @brief The XSDType enum
	 * Enumeration of the XSD types
	 */
	enum class XSDType {
		/** xsd:string */
		STRING = 0,
		/** xsd:boolean */
		BOOLEAN,
		/** xsd:double */
		DOUBLE,
		/** xsd:float */
		FLOAT,
		/** xsd:integer */
		INTEGER,
		/** xsd:long */
		LONG,
		/** xsd:short */
		SHORT,
		/** xsd:unsignedLong */
		UNSIGNED_LONG,
		/** xsd:unsignedInt */
		UNSIGNED_INT,
		/** xsd:unsignedShort */
		UNSIGNED_SHORT,
		/** xsd:nonNegativeInteger */
		NON_NEGATIVE_INTEGER,
		LAST // keep last
	};

	/**
	 * Get the XSD type from the IRI
	 * @param iri the IRI of the XSD type
	 * @return the XSD type
	 */
	XSDType xsdTypeFromIRI(std::string_view iri);

	/**
	 * Get the IRI of the XSD type
	 * @param type the XSD type
	 * @return the IRI of the XSD type
	 */
	std::string_view xsdTypeToIRI(XSDType type);
} // knowrob

#endif //KNOWROB_XSD_TYPE_H
