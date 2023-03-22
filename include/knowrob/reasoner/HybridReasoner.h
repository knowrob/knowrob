//
// Created by daniel on 12.03.23.
//

#ifndef KNOWROB_HYBRID_REASONER_H
#define KNOWROB_HYBRID_REASONER_H

namespace knowrob {
    /**
     * Evaluates a query by constructing a world graph and evaluating
     * the FOL query via graph matching.
     * A world representation is initially constructed from static knowledge and is further
     * completed via different methods.
     */
    class HybridReasoner {
    public:
        HybridReasoner();
    };

} // knowrob

#endif //KNOWROB_POSSIBLE_WORLD_REASONER_H
