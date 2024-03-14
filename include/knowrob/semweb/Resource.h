/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_SEMWEB_RESOURCE_H
#define KNOWROB_SEMWEB_RESOURCE_H

#include <string>
#include "knowrob/terms/Atom.h"
#include "knowrob/terms/IRIAtom.h"

namespace knowrob::semweb {
	/**
	 * A RDF resource.
	 */
	class Resource {
	public:
		explicit Resource(std::string_view iri);

		explicit Resource(const IRIAtomPtr &iri) : iri_(iri) {}

		/**
		 * @return the IRI string of this resource
		 */
		auto iri() const { return iri_->stringForm(); }

		/**
		 * @return the IRI string of this resource as an atom
		 */
		auto iriAtom() const { return iri_; }

		/**
		 * @return the name of this resource
		 */
		std::string_view name() const;

		/**
		 * @return the namespace of this resource.
		 */
		std::string_view ns(bool includeDelimiter=false) const;

		/**
		 * Generate a unique IRI for a resource.
		 * @param ns the namespace of the IRI.
		 * @param name the name prefix of the IRI.
		 * @return a unique IRI.
		 */
		static IRIAtomPtr unique_iri(std::string_view ns, std::string_view name);

		/**
		 * Generate a unique IRI for a resource.
		 * @param type_iri the type IRI of the resource used as a prefix.
		 * @return a unique IRI.
		 */
		static IRIAtomPtr unique_iri(std::string_view type_iri);

		/**
		 * @param iri a IRI
		 * @return the name part of the IRI
		 */
		static std::string_view iri_name(std::string_view iri);

		/**
		 * @param iri a IRI
		 * @param includeDelimiter if true, the delimiter is included in the result
		 * @return the namespace part of the IRI
		 */
		static std::string_view iri_ns(std::string_view iri, bool includeDelimiter=false);

	protected:
		knowrob::AtomPtr iri_;
	};

} // knowrob::semweb

#endif //KNOWROB_SEMWEB_RESOURCE_H
