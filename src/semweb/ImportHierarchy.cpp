//
// Created by daniel on 09.04.23.
//

#include <memory>
#include "knowrob/semweb/ImportHierarchy.h"

using namespace knowrob::semweb;

void ImportHierarchy::addCurrentGraph(const std::string_view &graphName)
{
    auto it = graphs_.find(graphName);
    if(it == graphs_.end()) {
        auto newGraph = new CurrentGraph(graphName);
        graphs_.emplace(newGraph->name(), newGraph);
    }
}

const std::set<CurrentGraph*>& ImportHierarchy::getImports(const std::string_view &graphName)
{
    auto it = graphs_.find(graphName);
    if(it == graphs_.end()) {
        static std::set<CurrentGraph*> empty;
        return empty;
    }
    else {
        return it->second->imports_;
    }
}

void ImportHierarchy::addDirectImport(const std::string_view &importerGraphName,
                                      const std::string_view &importedGraphName)
{
    // add nodes if not existing
    auto it1 = graphs_.find(importerGraphName);
    if(it1 != graphs_.end()) {
        auto newGraph = new CurrentGraph(importerGraphName);
        auto inserted = graphs_.emplace(newGraph->name(), newGraph);
        it1 = inserted.first;
    }
    auto it2 = graphs_.find(importedGraphName);
    if(it2 != graphs_.end()) {
        auto newGraph = new CurrentGraph(importedGraphName);
        auto inserted = graphs_.emplace(newGraph->name(), newGraph);
        it2 = inserted.first;
    }
    auto &g_importer = *it1->second;
    auto &g_imported = *it2->second;

    // g_importer.directlyImports += [g_imported]
    g_importer.directImports_.insert(&g_imported);
    // g_importer.imports += g_imported.imports + [gb]
    g_importer.imports_.insert(g_imported.imports_.begin(), g_imported.imports_.end());
    g_importer.imports_.insert(&g_imported);
    // for every graph gx that imports g_importer:
    //      gx.imports += (g_imported.imports + [g_imported])
    for(auto &it : graphs_) {
        if(it.second->imports_.count(&g_importer)>0) {
            it.second->imports_.insert(g_imported.imports_.begin(), g_imported.imports_.end());
            it.second->imports_.insert(&g_imported);
        }
    }
}

void ImportHierarchy::removeCurrentGraph(const std::string_view &graphName)
{
    auto it = graphs_.find(graphName);
    if(it == graphs_.end()) return;
    auto &ga = *it->second;

    // for every graph gx that directly imports ga
    for(auto &jt : graphs_) {
        auto &gx = *jt.second;
        if(gx.directImports_.count(&ga)>0) {
            // gx.directlyImports -= [ga]
            gx.directImports_.erase(&ga);
            gx.imports_.erase(&ga);
            // gx.directlyImports += [ga.directImports]
            // TODO: this is a bit questionable, probably better to not add ga'a direct imports to gx.
            gx.directImports_.insert(ga.directImports_.begin(), ga.directImports_.end());
        }
    }
    // for every graph gy that imports ga:
    for(auto &kt : graphs_) {
        auto &gy = *kt.second;
        if(gy.imports_.count(&ga)>0) {
            // gy.imports -= [ga]
            gy.imports_.erase(&ga);
        }
    }

    // finally delete the node
    graphs_.erase(it);
}
