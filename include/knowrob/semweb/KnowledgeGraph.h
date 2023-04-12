//
// Created by daniel on 12.03.23.
//

#ifndef KNOWROB_KNOWLEDGE_GRAPH_H
#define KNOWROB_KNOWLEDGE_GRAPH_H

#include "memory"
#include "optional"
#include "raptor2.h"
#include "knowrob/formulas/Literal.h"
#include "knowrob/queries/BufferedAnswerStream.h"
#include "GraphQuery.h"
#include "Vocabulary.h"
#include "TripleExpression.h"
#include "TripleData.h"

namespace knowrob {

    enum TripleFormat {
        RDF_XML,
        TURTLE,
        N_TRIPLES
    };

    class ITripleLoader {
    public:
        virtual void loadTriple(const TripleData &tripleData) = 0;
        virtual void flush() = 0;
    };

    class KnowledgeGraph {
    public:
        KnowledgeGraph();

        KnowledgeGraph(const KnowledgeGraph&) = delete;

        ~KnowledgeGraph();

        const auto& vocabulary() const { return vocabulary_; }

        bool isDefinedResource(const std::string_view &iri);

        bool isDefinedProperty(const std::string_view &iri);

        bool isDefinedClass(const std::string_view &iri);

        /**
         * Read RDF ontology from a remote URI or a local file, and load
         * triple data into the database backend.
         * @param uriString the URI pointing to a RDF file
         * @param format the format of the file
         * @return true if the file was loaded successfully
         */
        virtual bool loadTriples(const std::string_view &uriString, TripleFormat format) = 0;

        virtual void assertTriple(const TripleData &tripleData) = 0;

        virtual void assertTriples(const std::vector<TripleData> &tripleData) = 0;

        virtual void removeAllTriples(const semweb::TripleExpression &tripleExpression) = 0;

        virtual void removeOneTriple(const semweb::TripleExpression &tripleExpression) = 0;

        /**
         * Find instantiations of a literal in the knowledge graph.
         * @param query a graph query
         * @return a stream with answers to the query
         */
        virtual BufferedAnswerStreamPtr submitQuery(const GraphQueryPtr &query) = 0;

        /**
         * Watch for instantiations of a literal in the knowledge graph.
         * @param literal a literal
         * @return a stream with answers to the query
         */
        virtual BufferedAnswerStreamPtr watchQuery(const GraphQueryPtr &query) = 0;

        //virtual bool unwatchQuery(const BufferedAnswerStreamPtr &queryStream) = 0;

        /**
         * Ontologies are loaded into named sub-graphs of the knowledge graph.
         * The name is generated from the URI in case of loading RDF files.
         * @param uriString A URI pointing to a RDF ontology.
         * @return a graph name for the ontology
         */
        static std::string getNameFromURI(const std::string &uriString);

        /**
         * Get a version string from an ontology URI.
         * In case the URI points to a local file, the modification time of the file
         * is used as version.
         * For other URIs it is attempted to extract version information from the URI string,
         * if this fails, then the current day is used as a version string.
         * @param uriString A URI pointing to a RDF ontology.
         * @return a version string
         */
        static std::string getVersionFromURI(const std::string &uriString);

        static bool isVersionString(const std::string &versionString);

    protected:
        raptor_world *raptorWorld_;
        std::shared_ptr<semweb::Vocabulary> vocabulary_;

        bool loadURI(ITripleLoader &loader,
                     const std::string &uriString,
                     std::string &blankPrefix,
                     TripleFormat format);
    };

    using KnowledgeGraphPtr = std::shared_ptr<KnowledgeGraph>;

} // knowrob

#endif //KNOWROB_KNOWLEDGE_GRAPH_H
