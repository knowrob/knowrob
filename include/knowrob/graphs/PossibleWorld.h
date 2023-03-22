//
// Created by daniel on 12.03.23.
//

#ifndef KNOWROB_POSSIBLE_WORLD_H
#define KNOWROB_POSSIBLE_WORLD_H

#include "knowrob/graphs/KnowledgeGraph.h"
#include "knowrob/modalities/Modality.h"

namespace knowrob {

    /**
     * A possible world represents statements that are considered to be true with respect to a modality.
     * For example, temporal modality captures the world at different points in time.
     * The world is seen here as a relational structure represented by a knowledge graph.
     * I.e., entities in the world are considered as nodes, and relations between them as edges in the graph.
     */
    class PossibleWorld : public KnowledgeGraph {
    public:
        PossibleWorld() = default;
    };

} // knowrob

#endif //KNOWROB_POSSIBLE_WORLD_H
