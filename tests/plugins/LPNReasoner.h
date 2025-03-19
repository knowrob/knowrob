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
    };
} // knowrob

#endif //LPN_REASONER_H
