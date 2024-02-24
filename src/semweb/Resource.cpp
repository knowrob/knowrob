/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/semweb/Resource.h"
#include "knowrob/terms/IRIAtom.h"
#include "knowrob/terms/Blank.h"
#include "knowrob/Logger.h"

using namespace knowrob::semweb;

Resource::Resource(std::string_view iri) {
	switch (rdfNodeTypeGuess(iri)) {
		case RDFNodeType::BLANK:
			iri_ = Blank::Tabled(iri);
			break;
		case RDFNodeType::IRI:
			iri_ = IRIAtom::Tabled(iri);
			break;
		case RDFNodeType::LITERAL:
			KB_WARN("Resource created with literal value: {}. Treating as IRI.", iri);
			iri_ = Blank::Tabled(iri);
			break;
	}
}
