//
// Created by daniel on 12.03.23.
//

#ifndef KNOWROB_EPISTEMIC_MODALITY_H
#define KNOWROB_EPISTEMIC_MODALITY_H

#include "Modality.h"

namespace knowrob {
    /**
     * Epistemic modalities are concerned with knowledge and belief.
     */
    class EpistemicModality : public Modality {
    public:
        EpistemicModality() : Modality() {}
    };

} // knowrob

#endif //KNOWROB_EPISTEMIC_MODALITY_H
