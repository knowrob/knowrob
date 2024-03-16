/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_REIFICATION_CONTAINER_H
#define KNOWROB_REIFICATION_CONTAINER_H

#include <utility>

#include "knowrob/triples/TripleContainer.h"
#include "knowrob/semweb/Vocabulary.h"

namespace knowrob {
	using ReifiedNames = std::shared_ptr<std::vector<IRIAtomPtr>>;

	/**
	 * A container that reifies triples of an input container.
	 */
	class ReificationContainer : public semweb::TripleContainer {
	public:
		explicit ReificationContainer(semweb::TripleContainerPtr originalTriples,
									  semweb::VocabularyPtr vocabulary,
									  ReifiedNames reifiedNames);

		// Override TripleContainer
		ConstGenerator cgenerator() const override;

	protected:
		semweb::TripleContainerPtr originalTriples_;
		semweb::VocabularyPtr vocabulary_;
		ReifiedNames reifiedNames_;
	};
} // knowrob

#endif //KNOWROB_REIFICATION_CONTAINER_H
