//
// Created by daniel on 09.04.23.
//

#ifndef KNOWROB_SEMWEB_IMPORT_HIERARCHY_H
#define KNOWROB_SEMWEB_IMPORT_HIERARCHY_H

#include "string"
#include "set"
#include "map"

namespace knowrob::semweb {
    class CurrentGraph {
    public:
        explicit CurrentGraph(const std::string_view &name)
        : name_(name) {}

        const auto& name() const { return name_; }

        const auto& directImports() const { return directImports_; }

        const auto& imports() const { return imports_; }

    protected:
        const std::string name_;
        std::set<CurrentGraph*> directImports_;
        std::set<CurrentGraph*> imports_;
        friend class ImportHierarchy;
    };

    class ImportHierarchy {
    public:
        ImportHierarchy();

        bool isCurrentGraph(const std::string_view &graphName) const
        { return graphs_.count(graphName)>0; }

        void clear() { graphs_.clear(); }

        void setDefaultGraph(const std::string_view &defaultGraph) { defaultGraph_ = defaultGraph; }

        const auto& defaultGraph() const { return defaultGraph_; }

        void addCurrentGraph(const std::string_view &graphName);

        void removeCurrentGraph(const std::string_view &graphName);

        void addDirectImport(const std::string_view &importerGraphName,
                             const std::string_view &importedGraphName);

        const std::set<CurrentGraph*>& getImports(const std::string_view &graphName);

    protected:
        std::map<std::string_view, std::unique_ptr<CurrentGraph>> graphs_;
        std::string defaultGraph_;
    };

} // knowrob::semweb

#endif //KNOWROB_SEMWEB_IMPORT_HIERARCHY_H
