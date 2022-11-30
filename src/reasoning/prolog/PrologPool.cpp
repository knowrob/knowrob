/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include <knowrob/reasoning/prolog/PrologPool.h>

using namespace knowrob;

PrologPool::PrologPool(int num_initial_engines)
{
	for(int i=0; i<num_initial_engines; ++i) {
		available_engines_.push_back(
		    std::shared_ptr<PrologEngine>(new PrologEngine()));
	}
}

std::shared_ptr<PrologEngine> PrologPool::claim()
{
	std::lock_guard<std::mutex> scoped_lock(pool_mutex_);
	std::shared_ptr<PrologEngine> engine;
	if(available_engines_.empty()) {
		// simply create a new engine in case none is available at this time
		engine = std::shared_ptr<PrologEngine>(new PrologEngine());
	} else {
		engine = available_engines_.front();
		available_engines_.pop_front();
	}
	return engine;
}

void PrologPool::release(std::shared_ptr<PrologEngine> &engine)
{
	// make sure claim is lifted, and thread is in a good state
	engine->stopQuery(true);
	{
		std::lock_guard<std::mutex> scoped_lock(pool_mutex_);
		available_engines_.push_back(engine);
	}
}
