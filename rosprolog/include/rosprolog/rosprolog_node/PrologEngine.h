#ifndef __ROSPROLOG_ENGINE_H__
#define __ROSPROLOG_ENGINE_H__

// STD
#include <string>
#include <thread>
#include <mutex>
#include <list>
#include <condition_variable>
// boost
#include <boost/shared_ptr.hpp>
// SWI Prolog
#include <SWI-Prolog.h>

/**
 * Encapsulates an interface to a Prolog engine.
 * The engine runs in its own thread that can be claimed exclusively
 * by a query.
 * Different answers to the query can successively be generated.
 *
 * @author Daniel Beßler
 */
class PrologEngine {
public:
	PrologEngine();
	~PrologEngine();
	
	static int pl_exception(qid_t qid, std::string &exception_string);
	
	/**
	 * Convinience method that claims the engine, then computes
	 * one solution, and finally releases the engine again.
	 * Make sure the engine is not claimed otherwise before calling
	 * this method.
	 */
	boost::shared_ptr<std::string> one_solution(const std::string &query_string);
	
	/**
	 * Claims this engine, and assigns a goal.
	 * Make sure to release the engine again once you are done with it.
	 * Also make sure the engine is not claimed otherwise before calling
	 * this method.
	 */
	void claim(const std::string &goal, bool is_incremental);
	
	/**
	 * Lift the claim on this engine, making it available
	 * for others to submit a goal.
	 */
	void release(bool wait);
	
	/**
	 * True iff there is a next solution available.
	 * This call blocks until the next solution has been found,
	 * or the goal was finished otherwise.
	 */
	bool has_more_solutions();
	
	/**
	 * The next solution, if any.
	 */
	boost::shared_ptr<std::string> next_solution();
	
	bool has_error();
	
	const std::string& error();
	
private:
	std::thread thread_;
	std::mutex thread_m_;
	std::condition_variable thread_cv_;
	std::condition_variable solutions_cv_;
	std::condition_variable finished_cv_;
	
	bool is_claimed_;
	bool is_finished_;
	bool is_terminated_;
	bool is_incremental_;
	bool has_error_;
	bool has_terminate_request_;
	bool has_solution_request_;
	bool has_finish_request_;
	
	std::string goal_;
	
	std::string error_;
	
	std::list<boost::shared_ptr<std::string>> solutions_;
	
	boost::shared_ptr<std::string> pop_solution();
	
	void run();
};

class JSONPrologException : public std::exception
{
	std::string _msg;
public:
	JSONPrologException(const std::string& msg) : _msg(msg){}
	
	virtual const char* what() const noexcept override
	{
		return _msg.c_str();
	}
}; 

#endif //__ROSPROLOG_ENGINE_H__
