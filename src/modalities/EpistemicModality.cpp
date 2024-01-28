//
// Created by danielb on 01.08.23.
//

#define EPISTEMIC_MODALITY_AGENT_KEY "agent"

#include "knowrob/modalities/EpistemicModality.h"

using namespace knowrob;

EpistemicModality::EpistemicModality() : agent_(std::nullopt), Modality() {}

EpistemicModality::EpistemicModality(const std::string_view &agent)
        : agent_(Agent::get(agent)), Modality() {
    if(agent_.value() != Agent::getEgo()) {
        // TODO: could use a StringTerm with a sting_view (i.e. without copying).
        //       but that's not supported by StringTerm as of now...
        //       also agent could provide the term to avoid duplicates.
        parameters_[EPISTEMIC_MODALITY_AGENT_KEY] = std::make_shared<StringTerm>(agent_.value()->iri());
    }
}

const std::optional<AgentPtr>& EpistemicModality::agent() const { return agent_; }

ModalityType EpistemicModality::modalityType() const { return ModalityType::Epistemic; }

const char* EpistemicModality::necessity_symbol()   const { return "K"; }

const char* EpistemicModality::possibility_symbol() const { return "B"; }
