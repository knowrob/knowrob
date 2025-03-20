#ifndef LPN_REASONER_H
#define LPN_REASONER_H

#include <knowrob/reasoner/RDFGoalReasoner.h>

namespace knowrob {
    class LPNReasoner : public RDFGoalReasoner {
    public:
        explicit LPNReasoner(std::string_view pluginID);

        bool initializeReasoner(const PropertyTree& config) override;

        bool evaluate(RDFGoalPtr goal) override;

    private:
        IRIAtomPtr loves;

        bool evaluateJealous(const RDFGoalPtr &goal, const std::shared_ptr<TriplePattern> &triplePattern);

        bool evaluatePos(const RDFGoalPtr &goal, const std::shared_ptr<TriplePattern> &triplePattern);

        bool evaluateQuaternion(const RDFGoalPtr &goal, const std::shared_ptr<TriplePattern> &triplePattern);
    };
} // knowrob

#endif //LPN_REASONER_H
