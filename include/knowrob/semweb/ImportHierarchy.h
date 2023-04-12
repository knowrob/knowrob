//
// Created by daniel on 09.04.23.
//

#ifndef KNOWROB_SEMWEB_IMPORT_HIERARCHY_H
#define KNOWROB_SEMWEB_IMPORT_HIERARCHY_H

#include "string"
#include "set"
#include "map"

namespace knowrob::semweb {
    /**
     * A named graph currently defined in a knowledge graph.
     */
    class CurrentGraph {
    public:
        explicit CurrentGraph(const std::string_view &name)
        : name_(name) {}

        /**
         * @return the name of the graph.
         */
        const auto& name() const { return name_; }

        /**
         * @return list of direct imports of this graph.
         */
        const auto& directImports() const { return directImports_; }

        /**
         * @return transitive closure of imports relation starting from this graph.
         */
        const auto& imports() const { return imports_; }

    protected:
        const std::string name_;
        std::set<CurrentGraph*> directImports_;
        std::set<CurrentGraph*> imports_;
        friend class ImportHierarchy;
    };

    /**
     * Manages the import hierarchy between named graphs.
     * Names of graph usually correspond to names of ontology files,
     * and the import hierarchy is build based on owl:imports statements
     * in named graphs.
     */
    class ImportHierarchy {
    public:
        ImportHierarchy();

        /**
         * @param graphName a graph name
         * @return true if graphName is known in this hierarchy.
         */
        bool isCurrentGraph(const std::string_view &graphName) const;

        /**
         * Clear the hierarchy.
         */
        void clear() { graphs_.clear(); }

        /**
         * Set the default graph used for triples in case no graph name is specified.
         * @param defaultGraph a graph name.
         */
        void setDefaultGraph(const std::string_view &defaultGraph) { defaultGraph_ = defaultGraph; }

        /**
         * @return the default graph name of this hierarchy.
         */
        const auto& defaultGraph() const { return defaultGraph_; }

        /**
         * Defines a named graph if it is not defined yet.
         * @param graphName a graph name.
         */
        void addCurrentGraph(const std::string_view &graphName);

        /**
         * Removes any definition of a named graph.
         * @param graphName a graph name.
         */
        void removeCurrentGraph(const std::string_view &graphName);

        /**
         * Adds the imports relation between two named graphs.
         * @param importerGraphName a graph name.
         * @param importedGraphName a graph name.
         */
        void addDirectImport(const std::string_view &importerGraphName,
                             const std::string_view &importedGraphName);

        /**
         * @param graphName a graph name.
         * @return the transitive closure of the import relation starting from graphName
         */
        const std::set<CurrentGraph*>& getImports(const std::string_view &graphName);

    protected:
        std::map<std::string_view, std::unique_ptr<CurrentGraph>> graphs_;
        std::string defaultGraph_;
    };

} // knowrob::semweb

#endif //KNOWROB_SEMWEB_IMPORT_HIERARCHY_H
