/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_BLANK_H
#define KNOWROB_BLANK_H

#include "RDFNode.h"
#include "Atom.h"

namespace knowrob {
	/**
	 * A blank node is a node in an RDF graph that is neither a URI nor a literal.
	 */
	class Blank : public Atom, public RDFNode {
	public:
		/**
		 * Constructs a blank node from a name.
		 * @param name the name of the blank node
		 */
		explicit Blank(std::string_view name) : Atom(name) {}

		/**
		 * @param stringForm the string form of the IRI
		 * @return a shared pointer to an IRI atom
		 */
		static std::shared_ptr<Blank> Tabled(std::string_view stringForm);

		// Override RDFNode
		RDFNodeType rdfNodeType() const final { return RDFNodeType::BLANK; }

		// Override Term
		bool isBlank() const final { return true; }
	};

} // knowrob

#endif //KNOWROB_BLANK_H
