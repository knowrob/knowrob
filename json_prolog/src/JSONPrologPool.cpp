
#include <ros/ros.h>
#include <ros/console.h>
#include <json_prolog/JSONPrologPool.h>

JSONPrologPool::JSONPrologPool(int num_initial_engines)
{
	for(int i=0; i<num_initial_engines; ++i) {
		available_engines_.push_back(
		    boost::shared_ptr<JSONPrologEngine>(new JSONPrologEngine()));
	}
}

boost::shared_ptr<JSONPrologEngine> JSONPrologPool::claim()
{
	std::lock_guard<std::mutex> scoped_lock(pool_mutex_);
	boost::shared_ptr<JSONPrologEngine> engine;
	if(available_engines_.empty()) {
		// simply create a new engine in case none is available at this time
		engine = boost::shared_ptr<JSONPrologEngine>(new JSONPrologEngine());
	} else {
		engine = available_engines_.front();
		available_engines_.pop_front();
	}
	return engine;
}

void JSONPrologPool::release(boost::shared_ptr<JSONPrologEngine> &engine)
{
	// make sure claim is lifted, and thread is in a good state
	engine->release(true);
	{
		std::lock_guard<std::mutex> scoped_lock(pool_mutex_);
		available_engines_.push_back(engine);
	}
}
