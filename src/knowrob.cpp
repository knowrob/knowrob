/*
 * Copyright (c) 2023, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/knowrob.h>
#include <knowrob/Logger.h>

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
	}
}
