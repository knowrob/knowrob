/*
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <boost/python.hpp>
#include <knowrob/knowrob.h>
#include <knowrob/Logger.h>
#include <knowrob/ThreadPool.h>
#include <filesystem>
#include <iostream>
#include <locale>
#include "knowrob/py/PythonError.h"

uint32_t knowrob::GlobalSettings::batchSize_ = 500u;

knowrob::IRIAtomPtr knowrob::GlobalSettings::egoIRI_ =
	IRIAtom::Tabled("http://knowrob.org/kb/knowrob.owl#Self");

namespace knowrob {
	// stores the name of the executable as provided in argv[0]
	char *NAME_OF_EXECUTABLE = nullptr;

	char *getNameOfExecutable() {
		if (NAME_OF_EXECUTABLE) {
			return knowrob::NAME_OF_EXECUTABLE;
		} else {
			static char noExec[] = "<no-executable>";
			return noExec;
		}
	}

	void hashCombine(std::size_t &seed, const std::size_t &v) {
		static const auto GOLDEN_RATIO_HASH = static_cast<size_t>(0x9e3779b9);
		seed ^= v + GOLDEN_RATIO_HASH + (seed << 6) + (seed >> 2);
	}

	void InitPythonPath() {
		std::stringstream pythonPath;
		auto oldPath = std::getenv("PYTHONPATH");
		if (oldPath) {
			pythonPath << oldPath << ":";
		}
		pythonPath <<
				   (std::filesystem::path(KNOWROB_INSTALL_PREFIX) / "knowrob").string() << ":"
				   << KNOWROB_SOURCE_DIR << ":"
				   << KNOWROB_BUILD_DIR;
		auto pythonPathStr = pythonPath.str();
		KB_DEBUG("[KnowRob] using python path: {}", pythonPathStr);
		setenv("PYTHONPATH", pythonPathStr.c_str(), 1);
	}

	void InitKnowledgeBase(int argc, char **argv) {
		// remember the program name.
		// it is assumed here that argv stays valid during program execution.
		knowrob::NAME_OF_EXECUTABLE = argv[0];
		// set the locale to classic to avoid problems with number formatting,
		// especially regarding use of dot or comma as decimal separator.
		std::cout.imbue(std::locale::classic());
		// configure the logger
		Logger::initialize();
		// Allow Python to load modules KnowRob-related directories.
		InitPythonPath();
		// start a Python interpreter
		Py_Initialize();
		KB_INFO("[KnowRob] static initialization done.");
		KB_DEBUG("[KnowRob] executable: {}", getNameOfExecutable());
		KB_DEBUG("[KnowRob] source directory: {}", KNOWROB_SOURCE_DIR);
		KB_DEBUG("[KnowRob] install prefix: {}", KNOWROB_INSTALL_PREFIX);
		KB_DEBUG("[KnowRob] build directory: {}", KNOWROB_BUILD_DIR);
	}

	void ShutdownKnowledgeBase() {
		// NOTE: Py_Finalize() should not be called when using boost python according to docs.
		//Py_Finalize();
		// stop the thread pool, join all remaining threads
		DefaultThreadPool()->shutdown();
		KB_INFO("[KnowRob] shutdown complete.");
	}
}
