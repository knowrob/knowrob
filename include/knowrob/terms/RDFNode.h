/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_RDF_NODE_H
#define KNOWROB_RDF_NODE_H

#include <string>

namespace knowrob {
	/**
	 * The type of an RDF node.
	 */
	enum class RDFNodeType {
		/** blank node */
		BLANK,
		/** IRI */
		IRI,
		/** typed literal */
		LITERAL
	};

	/**
	 * An RDF node is an element of an RDF graph, i.e., a resource, a literal, or a blank node.
	 */
	class RDFNode {
	public:
		virtual RDFNodeType rdfNodeType() const = 0;
	};

	/**
	 * Guess the type of an RDF node from a string.
	 * @param str the string to guess the type from
	 * @return the guessed type
	 */
	RDFNodeType rdfNodeTypeGuess(std::string_view str);

} // knowrob

#endif //KNOWROB_RDF_NODE_H
