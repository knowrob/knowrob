//
// Created by daniel on 11.04.23.
//

#ifndef KNOWROB_SEMWEB_TRIPLE_DATA_H
#define KNOWROB_SEMWEB_TRIPLE_DATA_H

#include "optional"

namespace knowrob {
    enum RDFType {
        RDF_RESOURCE,
        RDF_STRING_LITERAL,
        RDF_DOUBLE_LITERAL,
        RDF_INT64_LITERAL,
        RDF_BOOLEAN_LITERAL
    };

    /**
     * Triple string data loaded from file.
     */
    struct TripleData {
        TripleData()
                : documentID(nullptr),
                  subject(nullptr),
                  predicate(nullptr),
                  object(nullptr),
                  graph(nullptr),
                  agent(nullptr),
                  objectDouble(0.0),
                  objectInteger(0),
                  begin(),
                  end(),
                  confidence(),
                  objectType(RDF_RESOURCE) {}
        TripleData(const char* subject,
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
                  begin(),
                  end(),
                  confidence(),
                  objectType(RDF_RESOURCE) {}
        void *documentID;
        const char* subject;
        const char *predicate;
        const char* object;
        const char* graph;
        const char* agent;
        std::optional<double> begin;
        std::optional<double> end;
        std::optional<double> confidence;
        double objectDouble;
        long objectInteger;
        RDFType objectType;
    };
}

#endif //KNOWROB_SEMWEB_TRIPLE_DATA_H
