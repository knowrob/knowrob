
#include <ros/ros.h>
#include <ros/console.h>
#include <rosprolog/rosprolog_node/PrologPool.h>

PrologPool::PrologPool(int num_initial_engines)
{
	for(int i=0; i<num_initial_engines; ++i) {
		available_engines_.push_back(
		    boost::shared_ptr<PrologEngine>(new PrologEngine()));
	}
}

boost::shared_ptr<PrologEngine> PrologPool::claim()
{
	std::lock_guard<std::mutex> scoped_lock(pool_mutex_);
	boost::shared_ptr<PrologEngine> engine;
	if(available_engines_.empty()) {
		// simply create a new engine in case none is available at this time
		engine = boost::shared_ptr<PrologEngine>(new PrologEngine());
	} else {
		engine = available_engines_.front();
		available_engines_.pop_front();
	}
	return engine;
}

void PrologPool::release(boost::shared_ptr<PrologEngine> &engine)
{
	// make sure claim is lifted, and thread is in a good state
	engine->release(true);
	{
		std::lock_guard<std::mutex> scoped_lock(pool_mutex_);
		available_engines_.push_back(engine);
	}
}
