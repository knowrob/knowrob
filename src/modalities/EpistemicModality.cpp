/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#define EPISTEMIC_MODALITY_AGENT_KEY "agent"

#include "knowrob/modalities/EpistemicModality.h"
#include "knowrob/terms/Atom.h"

using namespace knowrob;

EpistemicModality::EpistemicModality() : agent_(std::nullopt), Modality() {}

EpistemicModality::EpistemicModality(const std::string_view &agent)
		: agent_(Perspective::get(agent)), Modality() {
	if (Perspective::isEgoPerspective(agent_.value()->iri())) {
		parameters_[EPISTEMIC_MODALITY_AGENT_KEY] = Atom::Tabled(agent_.value()->iri());
	}
}

const std::optional<PerspectivePtr> &EpistemicModality::agent() const { return agent_; }

ModalityType EpistemicModality::modalityType() const { return ModalityType::Epistemic; }

const char *EpistemicModality::necessity_symbol() const { return "K"; }

const char *EpistemicModality::possibility_symbol() const { return "B"; }
