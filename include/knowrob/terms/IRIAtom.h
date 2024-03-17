/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_IRI_ATOM_H
#define KNOWROB_IRI_ATOM_H

#include "Atom.h"
#include "RDFNode.h"
#include "knowrob/formulas/Predicate.h"

namespace knowrob {
	/**
	 * An IRI (Internationalized Resource Identifier) within an RDF graph is a Unicode string
	 * that conforms to the syntax defined in RFC 3987.
	 */
	class IRIAtom : public Atom, public RDFNode {
	public:
		/**
		 * Constructs an IRI atom from a string.
		 * @param stringForm the string form of the IRI
		 */
		explicit IRIAtom(std::string_view stringForm) : Atom(stringForm) {}

		/**
		 * @param stringForm the string form of the IRI
		 * @return a shared pointer to an IRI atom
		 */
		static std::shared_ptr<IRIAtom> Tabled(std::string_view stringForm);

		/**
		 * Constructs a predicate from this IRI atom and the given terms.
		 * @param s the subject term
		 * @param o the object term
		 * @return a shared pointer to a predicate
		 */
		PredicatePtr operator()(const TermPtr &s, const TermPtr &o) const;

		// override Atom
		AtomType atomType() const override { return AtomType::IRI; }

		// override RDFNode
		RDFNodeType rdfNodeType() const override { return RDFNodeType::IRI; }

		// override Term
		bool isIRI() const override { return true; }

	protected:
		// override Term
		void write(std::ostream &os) const override;
	};

	using IRIAtomPtr = std::shared_ptr<IRIAtom>;

} // knowrob


#endif //KNOWROB_IRI_ATOM_H
