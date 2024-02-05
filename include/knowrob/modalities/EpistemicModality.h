//
// Created by daniel on 12.03.23.
//

#ifndef KNOWROB_EPISTEMIC_MODALITY_H
#define KNOWROB_EPISTEMIC_MODALITY_H

#include "knowrob/Logger.h"
#include "Modality.h"
#include "knowrob/terms/Constant.h"
#include "knowrob/semweb/Agent.h"

#include <utility>

namespace knowrob {
    enum class EpistemicOperator : uint8_t {
        KNOWLEDGE=1,
        BELIEF=2
    };

    /**
     * Epistemic is concerned with knowledge and belief.
     */
    class EpistemicModality : public Modality {
    public:
        EpistemicModality();

        explicit EpistemicModality(const std::string_view &agent);

        const std::optional<AgentPtr>& agent() const;

        ModalityType modalityType() const override;

        const char* necessity_symbol() const override;

        const char* possibility_symbol() const override;

    protected:
        const std::optional<AgentPtr> agent_;
    };

} // knowrob

#endif //KNOWROB_EPISTEMIC_MODALITY_H
