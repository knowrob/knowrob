/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_SEMWEB_RESOURCE_H
#define KNOWROB_SEMWEB_RESOURCE_H

#include <string>
#include "knowrob/terms/Atom.h"

namespace knowrob::semweb {
	/**
	 * A RDF resource.
	 */
	class Resource {
	public:
		explicit Resource(std::string_view iri);

		/**
		 * @return the IRI string of this resource
		 */
		auto iri() const { return iri_->stringForm(); }

	protected:
		knowrob::AtomPtr iri_;
	};

} // knowrob::semweb

#endif //KNOWROB_SEMWEB_RESOURCE_H
