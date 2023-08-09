//
// Created by daniel on 11.04.23.
//

#ifndef KNOWROB_SEMWEB_STATEMENT_DATA_H
#define KNOWROB_SEMWEB_STATEMENT_DATA_H

#include "optional"
#include "knowrob/modalities/TemporalModality.h"
#include "knowrob/modalities/EpistemicModality.h"

namespace knowrob {
	/**
	 * The type of a RDF resource or literal.
	 */
    enum RDFType {
        RDF_RESOURCE,
        RDF_STRING_LITERAL,
        RDF_DOUBLE_LITERAL,
        RDF_INT64_LITERAL,
        RDF_BOOLEAN_LITERAL
    };

    /**
     * A struct holding data that represents a statement in a knowledge graph.
     * The data is not copied, thus this struct can be used to map memory
     * allocated by an external library (like raptor) into datastructures used by KnowRob.
     * However, care should be taken that the StatementData is not used anymore
     * after the memory has been de-allocated in the external library.
     */
    struct StatementData {
        StatementData()
                : documentID(nullptr),
                  subject(nullptr),
                  predicate(nullptr),
                  object(nullptr),
                  graph(nullptr),
                  agent(nullptr),
                  objectDouble(0.0),
                  objectInteger(0),
                  objectType(RDF_RESOURCE) {}
        StatementData(const char* subject,
                      const char* predicate,
                      const char* object,
                      const char* graph="user",
                      const char* agent=nullptr)
                : documentID(nullptr),
                  subject(subject),
                  predicate(predicate),
                  object(object),
                  graph(graph),
                  agent(agent),
                  objectDouble(0.0),
                  objectInteger(0),
                  objectType(RDF_RESOURCE) {}
        void *documentID;
        const char* subject;
        const char *predicate;
        const char* object;
        const char* graph;
        const char* agent;
        std::optional<TemporalOperator> temporalOperator;
        std::optional<EpistemicOperator> epistemicOperator;
        std::optional<double> begin;
        std::optional<double> end;
        std::optional<double> confidence;
        double objectDouble;
        long objectInteger;
        RDFType objectType;
    };
}

#endif //KNOWROB_SEMWEB_STATEMENT_DATA_H
