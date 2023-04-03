//
// Created by daniel on 12.03.23.
//

#ifndef KNOWROB_KNOWLEDGE_GRAPH_H
#define KNOWROB_KNOWLEDGE_GRAPH_H

#include "memory"
#include "optional"
#include <raptor2.h>

namespace knowrob {

    enum RDFType {
        RDF_RESOURCE,
        RDF_STRING_LITERAL,
        RDF_DOUBLE_LITERAL,
        RDF_INT64_LITERAL,
        RDF_BOOLEAN_LITERAL
    };

    enum TripleFormat {
        RDF_XML,
        TURTLE,
        N_TRIPLES
    };

    /**
     * Triple string data loaded from file.
     * Subject and object can be blank nodes.
     */
    struct TripleData {
        TripleData() : subject(nullptr),
                       predicate(nullptr),
                       object(nullptr),
                       objectType(RDF_RESOURCE) {}
        const char* subject;
        const char *predicate;
        const char* object;
        RDFType objectType;
    };

    class TripleLoader {
    public:
        virtual void loadTriple(const TripleData &tripleData) = 0;
        virtual void flush() = 0;
        virtual void finish() = 0;
    };

    class KnowledgeGraph {
    public:
        KnowledgeGraph();

        KnowledgeGraph(const KnowledgeGraph&) = delete;

        ~KnowledgeGraph();

        static std::string getGraphNameFromURI(const std::string &uriString);
        static std::string getGraphVersionFromURI(const std::string &uriString);
        static bool isGraphVersionString(const std::string &versionString);

    protected:
        raptor_world *raptorWorld_;

        bool loadURI(TripleLoader &loader,
                     const std::string &uriString,
                     std::string &blankPrefix,
                     TripleFormat format);
    };

    using KnowledgeGraphPtr = std::shared_ptr<KnowledgeGraph>;

} // knowrob

#endif //KNOWROB_KNOWLEDGE_GRAPH_H
