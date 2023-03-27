//
// Created by daniel on 12.03.23.
//

#ifndef KNOWROB_KNOWLEDGE_GRAPH_H
#define KNOWROB_KNOWLEDGE_GRAPH_H

#include "memory"

namespace knowrob {

    class KnowledgeGraph {
    public:
        KnowledgeGraph() = default;
    };

    using KnowledgeGraphPtr = std::shared_ptr<KnowledgeGraph>;

} // knowrob

#endif //KNOWROB_KNOWLEDGE_GRAPH_H
