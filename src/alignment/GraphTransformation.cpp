/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "knowrob/alignment/GraphTransformation.h"

using namespace knowrob;

void GraphTransformation::pushOutputTriples(const TripleContainerPtr &triples) {
	if (nextTransformation_) {
		nextTransformation_->pushInputTriples(triples);
	} else if (next_) {
		next_(triples);
	} else {
		KB_WARN("No next transformation or handler set");
	}
}

void GraphTransformation::initializeNext() {
	if (nextTransformation_) {
		nextTransformation_->initializeTransformation();
	}
}

void GraphTransformation::finalizeNext() {
	if (nextTransformation_) {
		nextTransformation_->finalizeTransformation();
	}
}
