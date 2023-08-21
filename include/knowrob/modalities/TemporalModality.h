//
// Created by daniel on 12.03.23.
//

#ifndef KNOWROB_TIME_MODALITY_H
#define KNOWROB_TIME_MODALITY_H

#include "Modality.h"
#include "TimeInterval.h"

namespace knowrob {
    enum class TemporalOperator {
        ALWAYS=0,
        SOMETIMES=1
    };

    /**
     * A time modalFrame such as "past" or "future".
     */
    class TemporalModality : public Modality {
    public:
        TemporalModality();

        explicit TemporalModality(const TimeInterval &timeInterval);

        const std::optional<TimeInterval>& timeInterval() const;

    protected:
        const std::optional<TimeInterval> timeInterval_;
    };
} // knowrob

#endif //KNOWROB_TIME_MODALITY_H
