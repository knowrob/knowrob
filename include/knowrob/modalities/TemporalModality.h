//
// Created by daniel on 12.03.23.
//

#ifndef KNOWROB_TIME_MODALITY_H
#define KNOWROB_TIME_MODALITY_H

#include "Modality.h"
#include "TimeInterval.h"

namespace knowrob {
    /**
     * A time modality such as "past" or "future".
     */
    class TemporalModality : public Modality {
    public:
        TemporalModality()
        : timeInterval_(std::nullopt), Modality() {}

        explicit TemporalModality(const TimeInterval &timeInterval)
        : timeInterval_(timeInterval), Modality() {}

        const std::optional<TimeInterval>& timeInterval() const { return timeInterval_; }

    protected:
        const std::optional<TimeInterval> timeInterval_;
    };
} // knowrob

#endif //KNOWROB_TIME_MODALITY_H
