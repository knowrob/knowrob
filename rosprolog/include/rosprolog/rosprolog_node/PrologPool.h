#ifndef __ROSPROLOG_POOL_H__
#define __ROSPROLOG_POOL_H__

//STD
#include <string>
#include <list>
#include <mutex>
// BOOST
#include <boost/shared_ptr.hpp>
// rosprolog
#include <rosprolog/rosprolog_node/PrologEngine.h>

/**
 * A simple thread pool implementation for Prolog engines.
 * The pool starts at a user-defined size, but may grow in
 * case many parallel requests are issued.
 * 
 * @author Daniel Beßler
 */
class PrologPool {
public:
	PrologPool(int num_initial_engines);
	
	/**
	 * Claim a Prolog engine. This claim is exclusive.
	 * To allow others using the engine again, the
	 * claim needs to be lifted by calling *release*.
	 */
	boost::shared_ptr<PrologEngine> claim();
	
	/**
	 * Release the claim for an engine thread.
	 */
	void release(boost::shared_ptr<PrologEngine> &thread);

private:
	std::list< boost::shared_ptr<PrologEngine> > available_engines_;
	
	std::mutex pool_mutex_;
};

#endif //__ROSPROLOG_POOL_H__
