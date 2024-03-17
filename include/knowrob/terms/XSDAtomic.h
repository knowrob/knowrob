/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_XSD_ATOMIC_H
#define KNOWROB_XSD_ATOMIC_H

#include "string_view"
#include "Atomic.h"
#include "RDFNode.h"
#include "XSDType.h"

namespace knowrob {
	/**
	 * An atomic RDF node with an XSD datatype
	 */
	class XSDAtomic : public RDFNode, public Atomic {
	public:
		XSDAtomic() : RDFNode(), Atomic() {}

		/**
		 * Create an XSDAtomic from a lexical form and an XSD datatype IRI
		 * @param lexicalForm the lexical form of the RDF literal
		 * @param xsdTypeIRI the IRI of the XSD datatype
		 * @return a shared pointer to the XSDAtomic
		 */
		static std::shared_ptr<XSDAtomic> create(std::string_view lexicalForm, std::string_view xsdTypeIRI);

		/**
		 * The IRI of the XSD datatype
		 * @return the IRI of the XSD datatype of the RDF literal
		 */
		std::string_view xsdTypeIRI() const;

		/**
		 * The XSD datatype determines how the lexical form maps to a literal value
		 * @return the XSD datatype of the RDF literal
		 */
		virtual XSDType xsdType() const = 0;

		// override RDFNode
		RDFNodeType rdfNodeType() const override { return RDFNodeType::LITERAL; }
	};

} // knowrob

#endif //KNOWROB_XSD_ATOMIC_H
