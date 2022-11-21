/*
 * Copyright (c) 2022, Daniel Beßler
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

// TODO: get rid of imports
#include <ros/ros.h>
#include <ros/console.h>

#include <knowrob/reasoning/prolog/PrologEngine.h>

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
	std::string thread_id; {
		std::stringstream ss;
		ss << thread_.get_id();
		thread_id = ss.str();
	}
	ROS_DEBUG("[%s] PrologEngine thread started.", thread_id.c_str());

	if(!PL_thread_attach_engine(NULL)) {
		ROS_ERROR("PrologEngine failed to attach engine!");
		is_terminated_ = true;
		return;
	}
	//
	while(ros::ok()) {
		// wait for a claim
		{
			std::unique_lock<std::mutex> lk(thread_m_);
			thread_cv_.wait(lk, [this]{return has_terminate_request_ ||
			                                  (is_claimed_ && !is_finished_);});
			if(has_terminate_request_) break;
		}
		ROS_DEBUG("[%s] PrologEngine thread claimed.", thread_id.c_str());
		// create a query
		term_t a1 = PL_new_term_refs(2);
		term_t a2 = a1+1;
		PL_put_atom_chars(a1, goal_.c_str());
		//
		int flags = PL_Q_CATCH_EXCEPTION|PL_Q_NODEBUG;
		// TODO: need to re-think how answers are generated here.
		//        maybe it would be ok to stick to json encoding, and just
		//        parse it into Answer objects
		qid_t qid = PL_open_query(NULL,flags,
		    PL_pred(PL_new_functor(PL_new_atom("rosprolog_query"),2),NULL),
		    a1);
		// do the query processing
		while(ros::ok()) {
			// here is where the main work is done
			if(!PL_next_solution(qid)) {
				has_error_ = PrologEngine::pl_exception(qid,error_);
				break;
			}
			// read the JSON encoded solution into std:string
			atom_t atom;
			if(!PL_get_atom(a2,&atom)) {
				has_error_ = true;
				error_ = "failed to read result atom.";
				break;
			}
			boost::shared_ptr<Answer> solution =
			    boost::shared_ptr<Answer>(new std::string(PL_atom_chars(atom)));

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
		PL_reset_term_refs(a1);
		ROS_DEBUG("[%s] PrologEngine thread released.", thread_id.c_str());
	}

	ROS_DEBUG("[%s] PrologEngine thread terminated.", thread_id.c_str());
	finished_cv_.notify_all();
	solutions_cv_.notify_all();
	is_terminated_ = true;
	PL_thread_destroy_engine();
}

void PrologEngine::consult(const std::string &filePath)
{
}

void PrologEngine::assert(const Predicate &predicate)
{
    //int PL_assert(term_t t, module_t m, int flags)
}

void PrologEngine::assert(const Rule &rule)
{
}

boost::shared_ptr<Answer> PrologEngine::oneSolution(boost::shared_ptr<IQuery> &goal)
{
	boost::shared_ptr<Answer> solution;
	claim(goal,true);
	if(hasMoreSolutions()) {
		solution = popSolution();
	}
	release(true);
	return solution;
}

void PrologEngine::claim(boost::shared_ptr<IQuery> &goal, bool isIncremental)
{
	{
		std::lock_guard<std::mutex> lk(thread_m_);
		if(is_claimed_) {
			throw JSONPrologException("the engine is claimed by someone else.");
		}
		if(is_terminated_) {
			throw JSONPrologException("the engine is terminated.");
		}
		is_incremental_ = is_incremental;
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

void PrologEngine::release(bool wait)
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

boost::shared_ptr<Answer> PrologEngine::nextSolution()
{
	if(!hasMoreSolutions()) {
		throw JSONPrologException("no next solution.");
	}
	boost::shared_ptr<Answer> solution = popSolution();

	// pre-compute next solution
	{
		std::lock_guard<std::mutex> lk(thread_m_);
		has_solution_request_ = true;
	}
	thread_cv_.notify_one();

	return solution;
}

boost::shared_ptr<Answer> PrologEngine::popSolution()
{
	std::lock_guard<std::mutex> lk(thread_m_);
	boost::shared_ptr<Answer> head = solutions_.front();
	solutions_.pop_front();
	return head;
}
