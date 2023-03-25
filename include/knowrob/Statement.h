//
// Created by daniel on 06.03.23.
//

#ifndef KNOWROB_STATEMENT_H
#define KNOWROB_STATEMENT_H

#include "knowrob/formulas/Predicate.h"
#include "knowrob/modalities/TimeInterval.h"
#include "knowrob/modalities/ConfidenceValue.h"

namespace knowrob {

    class Statement {
    public:
        /**
         * @predicate a predicate without free variable.
         */
        explicit Statement(const PredicatePtr &predicate);

        const PredicatePtr& predicate() const { return predicate_; }

        /**
         * Assigns a time interval to this statement indicating that the
         * statement is only valid within this interval.
         * @param timeInterval the time interval of this query.
         */
        void setTimeInterval(const std::shared_ptr<TimeInterval> &timeInterval);

        /**
         * Assigns a confidence value to this statement indicating that it is
         * only true with a certain confidence.
         * @param confidenceValue the confidenceInterval interval of this query.
         */
        void setConfidenceValue(const std::shared_ptr<ConfidenceValue> &confidenceValue);

        /**
         * @return an optional time interval of this query.
         */
        const std::optional<const TimeInterval*>& timeInterval() const { return o_timeInterval_; }

        /**
         * @return an optional confidenceInterval interval of this query.
         */
        const std::optional<const ConfidenceValue*>& confidenceValue() const { return o_confidenceValue_; }

    protected:
        const PredicatePtr predicate_;
        std::shared_ptr<TimeInterval> timeInterval_;
        std::shared_ptr<ConfidenceValue> confidenceValue_;
        std::optional<const TimeInterval*> o_timeInterval_;
        std::optional<const ConfidenceValue*> o_confidenceValue_;
    };

} // knowrob

namespace std {
    std::ostream& operator<<(std::ostream& os, const knowrob::Statement& s);
}

#endif //KNOWROB_STATEMENT_H
