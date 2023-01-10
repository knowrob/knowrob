/*
 * Copyright (c) 2023, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_H_
#define KNOWROB_H_

namespace knowrob {
	/**
	 * Static initialization of the knowledge base.
	 * Note that it is important that argv[0] holds the name
	 * of the executable.
	 * @param argc number of arguments in argv.
	 * @param argv array of program arguments, argv[0] is the name of the binary.
	 */
	void InitKnowledgeBase(int argc, char **argv);

	/**
	 * @return the name of the executable in which the knowledge base is running.
	 */
	char* getNameOfExecutable();
}

#endif //KNOWROB_H_
