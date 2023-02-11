/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <SWI-Prolog.h>

#include "knowrob/Logger.h"
#include "knowrob/reasoner/prolog/PrologThreadPool.h"

using namespace knowrob;

PrologThreadPool::PrologThreadPool(uint32_t maxNumThreads)
: ThreadPool(maxNumThreads)
{}

bool PrologThreadPool::initializeWorker()
{
	// call PL_thread_attach_engine once initially for each worker thread
	if(PL_thread_attach_engine(nullptr) < 0) {
		// `-1` indicates an error, and `-2` that Prolog is compiled without multithreading support
		KB_ERROR("Failed to attach Prolog engine to current thread!");
		return false;
	}
	else {
		// if PL_thread_attach_engine()>0, then the Prolog ID for the thread was returned
		KB_DEBUG("Attached Prolog engine to current thread.");
		return true;
	}
}

void PrologThreadPool::finalizeWorker()
{
	// destroy the engine previously bound to this thread
	PL_thread_destroy_engine();
	KB_ERROR("destroyed Prolog engine");
}
