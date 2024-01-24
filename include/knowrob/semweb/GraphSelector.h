//
// Created by daniel on 17.01.24.
//

#ifndef KNOWROB_GRAPH_SELECTOR_H
#define KNOWROB_GRAPH_SELECTOR_H

#include "optional"
#include "knowrob/modalities/TemporalModality.h"
#include "knowrob/modalities/EpistemicModality.h"
#include "Agent.h"

namespace knowrob {
    struct GraphSelector {
        GraphSelector()
                : graph(nullptr) {}
        const char* graph; // TODO: rename to "ontology" ?
        std::optional<AgentPtr> agent;
        std::optional<TemporalOperator> temporalOperator;
        std::optional<EpistemicOperator> epistemicOperator;
        std::optional<double> begin;
        std::optional<double> end;
        std::optional<double> confidence;

		static const GraphSelector& getDefault()
		{
			static GraphSelector defaultSelector;
			return defaultSelector;
		}
    };
}

#endif //KNOWROB_GRAPH_SELECTOR_H
