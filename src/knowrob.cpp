/*
 * Copyright (c) 2023, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/knowrob.h>
#include <knowrob/Logger.h>
#include <Python.h>
#include <filesystem>
#include <iostream>
#include <locale>

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
		// TODO: rather add sub-paths? the structure with "src" does not fit well with python.
		setenv("PYTHONPATH", KNOWROB_INSTALL_PREFIX, 1);
		setenv("PYTHONPATH", KNOWROB_SOURCE_DIR, 1);
		setenv("PYTHONPATH", KNOWROB_BUILD_DIR, 1);
		// start a Python interpreter
		// NOTE: Py_Finalize() should not be called when using boost python according to docs.
		Py_Initialize();
	}

	// TODO: add a finalize function that makes sure worker threads are stopped, and joined.
	//		there is e.g. problem with spdlog being used in a worker thread after the main thread
	//      has already been exited (it seems).
}
