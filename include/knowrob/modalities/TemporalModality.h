//
// Created by daniel on 12.03.23.
//

#ifndef KNOWROB_TIME_MODALITY_H
#define KNOWROB_TIME_MODALITY_H

#include "Modality.h"

namespace knowrob {
    /**
     * A time modality such as "past" or "future".
     */
    class TemporalModality : public Modality {
    public:
        TemporalModality() : Modality() {}
    };
} // knowrob

#endif //KNOWROB_TIME_MODALITY_H
