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

namespace knowrob {
	// stores the name of the executable as provided in argv[0]
	char *NAME_OF_EXECUTABLE = nullptr;

	char* getNameOfExecutable()
	{
		if(NAME_OF_EXECUTABLE) {
			return knowrob::NAME_OF_EXECUTABLE;
		} else {
			static char noExec[] = "<no-executable>";
			return noExec;
		}
	}

	void InitKnowledgeBase(int argc, char **argv)
	{
		// remember the program name.
		// it is assumed here that argv stays valid during program execution.
		knowrob::NAME_OF_EXECUTABLE = argv[0];
		// configure the logger
		Logger::initialize();
		// Allow Python to load modules KnowRob-related directories.
		// TODO: rather add sub-paths? the structure with "src" does not fit well with python.
		setenv("PYTHONPATH", KNOWROB_INSTALL_PREFIX, 1);
		setenv("PYTHONPATH", KNOWROB_SOURCE_DIR, 1);

		auto buildPath = std::filesystem::path(KNOWROB_SOURCE_DIR) / "cmake-build-debug"; // FIXME
		setenv("PYTHONPATH", buildPath.c_str(), 1);
		// start a Python interpreter
		// NOTE: Py_Finalize() should not be called when using boost python according to docs.
		Py_Initialize();
	}
}
