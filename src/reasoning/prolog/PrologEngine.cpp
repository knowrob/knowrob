/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

// STD
#include <sstream>
// SPDLOG
#include <spdlog/spdlog.h>
// KnowRob
#include <knowrob/knowrob.h>
#include <knowrob/reasoning/prolog/PrologEngine.h>
#include <knowrob/reasoning/prolog/PrologQuery.h>

using namespace knowrob;

PrologEngine::PrologEngine() :
	is_claimed_(false),
	is_finished_(false),
	is_terminated_(false),
	is_incremental_(false),
	has_terminate_request_(false),
	has_solution_request_(false),
	has_finish_request_(false),
	has_error_(false),
	thread_(&PrologEngine::run, this),
	error_("")
{
}

PrologEngine::~PrologEngine()
{
	{
		std::unique_lock<std::mutex> lk(thread_m_);
		has_terminate_request_ = true;
	}
	thread_cv_.notify_one();
	thread_.join();
}

int PrologEngine::plException(qid_t qid, std::string &x)
{
	term_t except;
	char *except_msg = NULL;
	if(!(except = PL_exception(qid))) {
		return FALSE;
	}
	if(PL_get_chars(except, &except_msg, CVT_WRITE | BUF_RING)) {
		// TODO: also JSON encode the error? Would require to change
		//        all the clients though.
		x = std::string(except_msg);
	}
	PL_clear_exception();
	return TRUE;
}

void PrologEngine::run()
{
	static const int flags = PL_Q_CATCH_EXCEPTION|PL_Q_NODEBUG;
	
	std::shared_ptr<QueryResult> solution;
	std::string thread_id; {
		std::stringstream ss;
		ss << thread_.get_id();
		thread_id = ss.str();
	}
	spdlog::debug("PrologEngine {} thread started.", thread_id.c_str());

	if(!PL_thread_attach_engine(NULL)) {
		spdlog::error("PrologEngine {} failed to attach engine!", thread_id.c_str());
		is_terminated_ = true;
		return;
	}
	//
	while(knowrob::isRunning()) {
		// wait for a claim
		{
			std::unique_lock<std::mutex> lk(thread_m_);
			thread_cv_.wait(lk, [this]{return has_terminate_request_ ||
			                                  (is_claimed_ && !is_finished_);});
			if(has_terminate_request_) break;
		}
		spdlog::debug("PrologEngine {} thread claimed.", thread_id.c_str());
		
		// TODO: INPUT QUEUE
		//	- pull out results until end-of-queue message
		//	- implement goal_->applySubstitution replacing variables
		//           create a new goal each time for now
		// TODO: more efficient way with less than n calls of PL_open_query for n inputs?
		//	- one could bind an engine to each input to make it multithreaded.
		//        but would create massive amounts of threads if number is not capped.
		//      - else processing is strictly sequential, but maybe it's possible to combine
		//        all queued input in each step.
		//	  Prolog has lazy lists that could be used with some effort, but is it worth it?
		//        processing would still be sequential but only one PL_open_query call
		//
		
		// create a query
		PrologQuery pl_query(goal_);
		//
		// TODO: is it really needed to wrap into call?
		qid_t qid = PL_open_query(NULL,flags,
		    PL_pred(PL_new_functor(PL_new_atom("call"),1),NULL),
		    pl_query.getPrologTerm());
		// do the query processing
		while(true) {
			// here is where the main work is done
			if(!PL_next_solution(qid)) {
				has_error_ = PrologEngine::plException(qid,error_);
				break;
			}
			
			// TODO: use input mapping
			// construct a query result
			solution = std::shared_ptr<QueryResult>(new QueryResult);
			for(const auto& kv: pl_query.vars()) {
				PrologQuery x(kv.second);
				const auto &phi = x.getQuery()->formula();
				if(phi->type() == FormulaType::PREDICATE) {
					solution->set(kv.first, ((PredicateFormula*)phi.get())->predicate());
				}
				else {
					// TODO: how to handle comma and semicolon here?
				}
			}
			
			// notify that a new solution has been found
			{
				std::lock_guard<std::mutex> lk(thread_m_);
				if(has_terminate_request_) break;
				solutions_.push_back(solution);
			}
			solutions_cv_.notify_one();

			// wait on next solution request
			{
				std::unique_lock<std::mutex> lk(thread_m_);
				if(is_incremental_) {
					thread_cv_.wait(lk, [this]{return has_solution_request_ ||
									has_terminate_request_ ||
									has_finish_request_;});
					has_solution_request_ = false;
				}
				if(has_terminate_request_) break;
				if(has_finish_request_) break;
			}
		}
		{
			std::unique_lock<std::mutex> lk(thread_m_);
			is_finished_ = true;
			has_finish_request_ = false;
		}
		finished_cv_.notify_all();
		solutions_cv_.notify_all();
		PL_close_query(qid);
		spdlog::debug("PrologEngine {} thread released.", thread_id.c_str());
	}

	spdlog::debug("PrologEngine {} thread terminated.", thread_id.c_str());
	finished_cv_.notify_all();
	solutions_cv_.notify_all();
	is_terminated_ = true;
	PL_thread_destroy_engine();
}

bool PrologEngine::consult(const std::string &prologFilePath)
{
	// run query user:consult(prologFilePath)"
	return oneSolution(std::shared_ptr<Query>(new Query(
		std::shared_ptr<Predicate>(new Predicate(
			"user:consult", std::vector<std::shared_ptr<Term>>{
				std::shared_ptr<Term>(new StringAtom(prologFilePath))
			}
		))
	)))->hasSolution();
}

bool PrologEngine::assertFact(const std::shared_ptr<Predicate> &predicate)
{
	// run query "user:assertz(p(...))"
	return oneSolution(std::shared_ptr<Query>(new Query(
		std::shared_ptr<Predicate>(new Predicate(
			"user:assertz",
			std::vector<std::shared_ptr<Term>>{ predicate }
		))
	)))->hasSolution();
}

std::shared_ptr<QueryResult> PrologEngine::oneSolution(const std::shared_ptr<Query> &goal)
{
	std::shared_ptr<QueryResult> solution;
	startQuery(goal,true);
	if(hasMoreSolutions()) {
		solution = popSolution();
	}
	stopQuery(true);
	return solution;
}

void PrologEngine::startQuery(const std::shared_ptr<Query> &goal, bool isIncremental)
{
	{
		std::lock_guard<std::mutex> lk(thread_m_);
		if(is_claimed_) {
			throw PrologException("the engine is claimed by someone else.");
		}
		if(is_terminated_) {
			throw PrologException("the engine is terminated.");
		}
		is_incremental_ = isIncremental;
		goal_ = goal;
		// init flags for new claim
		is_claimed_ = true;
		is_finished_ = false;
		has_solution_request_ = false;
		has_finish_request_ = false;
		has_error_ = false;
	}
	// notify the thread that a claim has been made
	thread_cv_.notify_one();
}

void PrologEngine::stopQuery(bool wait)
{
	{
		std::lock_guard<std::mutex> lk(thread_m_);
		if(!is_claimed_) return;
		// first set the "finish request" flag
		has_finish_request_ = true;
	}
	// then notify the thread
	thread_cv_.notify_one();
	if(wait){
		std::unique_lock<std::mutex> lk(thread_m_);
		// and wait until the claim has been lifted
		finished_cv_.wait(lk, [this]{return is_terminated_ ||
		                                    is_finished_;});
		is_claimed_ = false;
		solutions_.clear();
	}
}

bool PrologEngine::hasError()
{
	return has_error_;
}

const std::string& PrologEngine::error()
{
	return error_;
}

bool PrologEngine::hasMoreSolutions()
{
	{
		std::unique_lock<std::mutex> lk(thread_m_);
		if(!is_claimed_) return false;
		// wait until a solution has been computed
		solutions_cv_.wait(lk, [this]{return is_terminated_ ||
		                                     is_finished_ ||
		                                     !solutions_.empty();});
	}
	return !solutions_.empty();
}

std::shared_ptr<QueryResult> PrologEngine::nextSolution()
{
	if(!hasMoreSolutions()) {
		throw PrologException("no next solution.");
	}
	std::shared_ptr<QueryResult> solution = popSolution();

	// pre-compute next solution
	{
		std::lock_guard<std::mutex> lk(thread_m_);
		has_solution_request_ = true;
	}
	thread_cv_.notify_one();

	return solution;
}

std::shared_ptr<QueryResult> PrologEngine::popSolution()
{
	std::lock_guard<std::mutex> lk(thread_m_);
	std::shared_ptr<QueryResult> head = solutions_.front();
	solutions_.pop_front();
	return head;
}

