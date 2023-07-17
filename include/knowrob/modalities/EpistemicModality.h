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
        explicit EpistemicModality(const std::optional<std::string> &agent)
        : agent_(agent), Modality() {}

        const std::optional<std::string>& agent() const { return agent_; }

    protected:
        const std::optional<std::string> agent_;
    };

} // knowrob

#endif //KNOWROB_EPISTEMIC_MODALITY_H
