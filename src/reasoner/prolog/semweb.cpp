/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#define PL_SAFE_ARG_MACROS

#include <SWI-cpp.h>

#include "knowrob/reasoner/prolog/semweb.h"
#include "knowrob/reasoner/prolog/PrologReasoner.h"
#include "knowrob/reasoner/ReasonerManager.h"
#include "knowrob/semweb/ImportHierarchy.h"
#include "knowrob/URI.h"

using namespace knowrob;

static inline std::shared_ptr<ImportHierarchy> getImportHierarchy(term_t t_manager, term_t t_reasoner) {
	static std::shared_ptr<ImportHierarchy> null;
	auto definedReasoner =
			PrologReasoner::getDefinedReasoner(t_manager, t_reasoner);
	if (!definedReasoner) return null;
	auto prologReasoner =
			std::dynamic_pointer_cast<PrologReasoner>(definedReasoner->reasoner());
	return prologReasoner ? prologReasoner->importHierarchy() : null;
}

static inline KnowledgeBase *getKnowledgeBase(term_t t_manager, term_t t_reasoner) {
	auto definedReasoner =
			PrologReasoner::getDefinedReasoner(t_manager, t_reasoner);
	if (!definedReasoner) return nullptr;
	return definedReasoner->reasoner()->reasonerManager().kb();
}

foreign_t sw_current_graph3(term_t t_manager, term_t t_reasoner, term_t t_graph) {
	auto hierarchy = getImportHierarchy(t_manager, t_reasoner);

	char *graph;
	if (hierarchy && PL_get_atom_chars(t_graph, &graph)) {
		return hierarchy->isCurrentGraph(graph);
	} else {
		return false;
	}
}

foreign_t sw_set_current_graph3(term_t t_manager, term_t t_reasoner, term_t t_graph) {
	auto hierarchy = getImportHierarchy(t_manager, t_reasoner);

	char *graph;
	if (hierarchy && PL_get_atom_chars(t_graph, &graph)) {
		hierarchy->addCurrentGraph(graph);
		return true;
	} else {
		return false;
	}
}

foreign_t sw_unset_current_graph3(term_t t_manager, term_t t_reasoner, term_t t_origin) {
	auto kb = getKnowledgeBase(t_manager, t_reasoner);
	if(!kb) return false;

	char *origin;
	if (PL_get_atom_chars(t_origin, &origin)) {
		return kb->removeAllWithOrigin(origin);
	} else {
		return false;
	}
}

foreign_t sw_graph_add_direct_import4(term_t t_manager, term_t t_reasoner, term_t t_importer, term_t t_imported) {
	auto hierarchy = getImportHierarchy(t_manager, t_reasoner);

	char *importer, *imported;
	if (hierarchy &&
		PL_get_atom_chars(t_importer, &importer) &&
		PL_get_atom_chars(t_imported, &imported)) {
		hierarchy->addDirectImport(importer, imported);
		return true;
	} else {
		return false;
	}
}

foreign_t sw_graph_get_imports4(term_t t_manager, term_t t_reasoner, term_t t_importer, term_t t_importedList) {
	auto hierarchy = getImportHierarchy(t_manager, t_reasoner);
	char *importer;
	if (hierarchy && PL_get_atom_chars(t_importer, &importer)) {
		PlTail l(t_importedList);
		for (auto &x: hierarchy->getImports(importer))
			l.append(x->name().c_str());
		l.close();
		return true;
	}
	return false;
}

foreign_t sw_set_default_graph3(term_t t_manager, term_t t_reasoner, term_t t_graph) {
	auto hierarchy = getImportHierarchy(t_manager, t_reasoner);
	char *graph;
	if (hierarchy && PL_get_atom_chars(t_graph, &graph)) {
		hierarchy->setDefaultGraph(graph);
		return true;
	}
	return false;

}

foreign_t sw_default_graph3(term_t t_manager, term_t t_reasoner, term_t t_graph) {
	auto hierarchy = getImportHierarchy(t_manager, t_reasoner);
	return hierarchy && PL_unify_atom_chars(t_graph, hierarchy->defaultGraph().c_str());
}

foreign_t sw_origin_any1(term_t t_origin) {
	return PL_unify_atom_chars(t_origin, ImportHierarchy::ORIGIN_ANY.data());
}

foreign_t sw_origin_system1(term_t t_origin) {
	return PL_unify_atom_chars(t_origin, ImportHierarchy::ORIGIN_SYSTEM.data());
}

foreign_t sw_origin_session1(term_t t_origin) {
	return PL_unify_atom_chars(t_origin, ImportHierarchy::ORIGIN_SESSION.data());
}

foreign_t sw_origin_user1(term_t t_origin) {
	return PL_unify_atom_chars(t_origin, ImportHierarchy::ORIGIN_USER.data());
}

foreign_t sw_origin_reasoner1(term_t t_origin) {
	return PL_unify_atom_chars(t_origin, ImportHierarchy::ORIGIN_REASONER.data());
}

foreign_t sw_origin_test1(term_t t_origin) {
	return PL_unify_atom_chars(t_origin, ImportHierarchy::ORIGIN_TEST.data());
}

foreign_t sw_url_graph2(term_t t_url, term_t t_graph) {
	char *url;
	if (PL_get_atom_chars(t_url, &url)) {
		auto name = DataSource::getNameFromURI(url);
		return PL_unify_atom_chars(t_graph, name.c_str());
	}
	return false;
}

foreign_t sw_url_version2(term_t t_url, term_t t_version) {
	char *url;
	if (PL_get_atom_chars(t_url, &url)) {
		auto version = DataSource::getVersionFromURI(url);
		return PL_unify_atom_chars(t_version, version.c_str());
	}
	return false;
}

foreign_t sw_load_rdf_xml4(term_t t_manager, term_t t_reasoner, term_t t_url, term_t t_parentGraph) {
	auto kb = getKnowledgeBase(t_manager, t_reasoner);
	char *url, *parentGraph;
	if (kb && PL_get_atom_chars(t_url, &url) && PL_get_atom_chars(t_parentGraph, &parentGraph)) {
		URI ontologyURI(url);
		auto ontologyFile = std::make_shared<OntologyFile>(ontologyURI, "rdf-xml");
		ontologyFile->setParentOrigin(parentGraph);
		if (kb->loadDataSource(ontologyFile)) {
			return true;
		}
	}
	return false;
}

namespace knowrob::prolog {
	PL_extension PL_extension_semweb[] = {
			{"sw_url_graph",                   2, (pl_function_t) sw_url_graph2,                0},
			{"sw_url_version",                 2, (pl_function_t) sw_url_version2,              0},
			{"sw_default_graph_cpp",           3, (pl_function_t) sw_default_graph3,            0},
			{"sw_origin_any",                  1, (pl_function_t) sw_origin_any1,               0},
			{"sw_origin_system",               1, (pl_function_t) sw_origin_system1,            0},
			{"sw_origin_session",              1, (pl_function_t) sw_origin_session1,           0},
			{"sw_origin_user",                 1, (pl_function_t) sw_origin_user1,              0},
			{"sw_origin_reasoner",             1, (pl_function_t) sw_origin_reasoner1,          0},
			{"sw_origin_test",                 1, (pl_function_t) sw_origin_test1,              0},
			{"sw_set_default_graph_cpp",       3, (pl_function_t) sw_set_default_graph3,        0},
			{"sw_graph_get_imports_cpp",       4, (pl_function_t) sw_graph_get_imports4,       0},
			{"sw_graph_add_direct_import_cpp", 4, (pl_function_t) sw_graph_add_direct_import4, 0},
			{"sw_current_graph_cpp",           3, (pl_function_t) sw_current_graph3,           0},
			{"sw_set_current_graph_cpp",       3, (pl_function_t) sw_set_current_graph3,       0},
			{"sw_unset_current_graph_cpp",     3, (pl_function_t) sw_unset_current_graph3,     0},
			{"sw_load_rdf_xml_cpp",            4, (pl_function_t) sw_load_rdf_xml4,            0},
			{nullptr,                          0, nullptr,                                     0}
	};
}
