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
	/**
	 * The data base can contain multiple graphs, and this selector
	 * is used to select a subset of them.
	 * For example, each point in time is conceptually a separate graph, and
	 * a query may only address a specific point in time, or time interval.
	 */
    struct GraphSelector {
        GraphSelector() : graph(nullptr) {}
		/**
		 * The name of the graph, usually reflects the name of an ontology.
		 */
        const char* graph; // TODO: rename to "ontology" ?
        /**
         * The agent that is the host of the knowledge.
         */
        std::optional<AgentPtr> agent;
        /**
         * The temporal modality of consideration.
         */
        std::optional<TemporalOperator> temporalOperator;
        /**
         * The epistemic modality of consideration.
         */
        std::optional<EpistemicOperator> epistemicOperator;
        /**
         * The begin of the time interval of consideration.
         */
        std::optional<double> begin;
        /**
         * The end of the time interval of consideration.
         */
        std::optional<double> end;
        /**
         * The minimum confidence threshold for statements.
         */
        std::optional<double> confidence;

		static const GraphSelector& getDefault()
		{
			static GraphSelector defaultSelector;
			return defaultSelector;
		}
    };
}

#endif //KNOWROB_GRAPH_SELECTOR_H
