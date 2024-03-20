/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <memory>
#include "knowrob/semweb/ImportHierarchy.h"
#include "knowrob/Logger.h"

using namespace knowrob;

ImportHierarchy::ImportHierarchy()
		: defaultGraph_(ORIGIN_USER) {
	addDirectImport(ORIGIN_ANY, ORIGIN_SYSTEM);
	addDirectImport(ORIGIN_ANY, ORIGIN_SESSION);
	addDirectImport(ORIGIN_SESSION, ORIGIN_USER);
	addDirectImport(ORIGIN_SESSION, ORIGIN_REASONER);
	addDirectImport(ORIGIN_SESSION, ORIGIN_TEST);
}

bool ImportHierarchy::isReservedOrigin(std::string_view origin) {
	static std::set<std::string_view> reservedOrigins = {ORIGIN_SYSTEM, ORIGIN_SESSION, ORIGIN_USER, ORIGIN_REASONER, ORIGIN_TEST};
	return reservedOrigins.count(origin) > 0;
}

bool ImportHierarchy::isCurrentGraph(std::string_view graphName) const {
	return graphs_.count(graphName) > 0;
}

bool ImportHierarchy::isSystemOrigin(CurrentGraph &graph) {
	auto originSystem = getCurrentGraph(ORIGIN_SYSTEM);
	return graph == originSystem || originSystem.imports_.count(&graph) > 0;
}

bool ImportHierarchy::isSessionOrigin(CurrentGraph &graph) {
	auto originSession = getCurrentGraph(ORIGIN_SESSION);
	return graph == originSession || originSession.imports_.count(&graph) > 0;
}

void ImportHierarchy::addCurrentGraph(std::string_view graphName) {
	auto it = graphs_.find(graphName);
	if (it == graphs_.end()) {
		auto newGraph = new CurrentGraph(graphName);
		graphs_.emplace(newGraph->name(), newGraph);
	}
}

const std::set<CurrentGraph *> &ImportHierarchy::getImports(std::string_view graphName) {
	auto it = graphs_.find(graphName);
	if (it == graphs_.end()) {
		static std::set<CurrentGraph *> empty;
		return empty;
	} else {
		return it->second->imports_;
	}
}

CurrentGraph& ImportHierarchy::getCurrentGraph(std::string_view name) {
	auto it1 = graphs_.find(name);
	if (it1 == graphs_.end()) {
		auto newGraph = new CurrentGraph(name);
		auto inserted = graphs_.emplace(newGraph->name(), newGraph);
		it1 = inserted.first;
	}
	return *it1->second;
}

void ImportHierarchy::addDirectImport(std::string_view importerGraphName, std::string_view importedGraphName) {
	auto &g_importer = getCurrentGraph(importerGraphName);
	auto &g_imported = getCurrentGraph(importedGraphName);

	// if a system graph imports a session graph, remove the session graph from the session
	// before adding to the system. And if a session graph imports a system graph, ignore the import.
	if (isSystemOrigin(g_importer) && isSessionOrigin(g_imported)) {
		removeCurrentGraph(importedGraphName);
	} else if (isSessionOrigin(g_importer) && isSystemOrigin(g_imported)) {
		KB_WARN("Ignoring session graph \"{}\" import of system graph \"{}\".", importerGraphName, importedGraphName);
		return;
	}

	KB_DEBUG("Graph \"{}\" imports \"{}\".", importerGraphName, importedGraphName);
	// g_importer.directlyImports += [g_imported]
	g_importer.directImports_.insert(&g_imported);
	// g_importer.imports += g_imported.imports + [gb]
	g_importer.imports_.insert(g_imported.imports_.begin(), g_imported.imports_.end());
	g_importer.imports_.insert(&g_imported);
	// for every graph gx that imports g_importer:
	//      gx.imports += (g_imported.imports + [g_imported])
	for (auto &it: graphs_) {
		if (it.second->imports_.count(&g_importer) > 0) {
			it.second->imports_.insert(g_imported.imports_.begin(), g_imported.imports_.end());
			it.second->imports_.insert(&g_imported);
		}
	}
}

void ImportHierarchy::removeCurrentGraph(std::string_view graphName) {
	auto it = graphs_.find(graphName);
	if (it == graphs_.end()) return;
	auto &ga = *it->second;

	// for every graph gx that directly imports ga
	for (auto &jt: graphs_) {
		auto &gx = *jt.second;
		if (gx.directImports_.count(&ga) > 0) {
			// gx.directlyImports -= [ga]
			gx.directImports_.erase(&ga);
			gx.imports_.erase(&ga);
			// gx.directlyImports += [ga.directImports]
			gx.directImports_.insert(ga.directImports_.begin(), ga.directImports_.end());
		}
	}
	// for every graph gy that imports ga:
	for (auto &kt: graphs_) {
		auto &gy = *kt.second;
		if (gy.imports_.count(&ga) > 0) {
			// gy.imports -= [ga]
			gy.imports_.erase(&ga);
		}
	}

	// finally delete the node
	graphs_.erase(it);
}
