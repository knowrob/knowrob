//
// Created by daniel on 12.03.23.
//

#ifndef KNOWROB_EPISTEMIC_MODALITY_H
#define KNOWROB_EPISTEMIC_MODALITY_H

#include "Modality.h"

#include <utility>

namespace knowrob {
    /**
     * Epistemic is concerned with knowledge and belief.
     */
    class EpistemicModality : public Modality {
    public:
        EpistemicModality() : agent_(std::nullopt), Modality() {}

        explicit EpistemicModality(const std::string_view &agent)
        : agent_(agent), Modality() {}

        const std::optional<std::string>& agent() const { return agent_; }

        ModalityType modalityType() const override { return ModalityType::Epistemic; }

        const char* necessity_symbol()   const override { return "K"; }

        const char* possibility_symbol() const override { return "B"; }

    protected:
        const std::optional<std::string> agent_;
    };

} // knowrob

#endif //KNOWROB_EPISTEMIC_MODALITY_H
