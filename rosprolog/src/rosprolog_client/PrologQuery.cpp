/* 
 * Copyright (c) 2010, Lorenz Moesenlechner <moesenle@cs.tum.edu>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <boost/lexical_cast.hpp>

#include <ros/ros.h>

#include <rosprolog/rosprolog_client/PrologQuery.h>
#include <rosprolog/rosprolog_client/PrologClient.h>

#include <json_prolog_msgs/PrologQuery.h>
#include <json_prolog_msgs/PrologNextSolution.h>
#include <json_prolog_msgs/PrologFinish.h>

void PrologQuery::iterator::increment()
{
	if(!query_ || data_ == query_->bindings_.end())
	{
		throw PrologQuery::QueryError("Cannot increment a query end iterator.");
	}
	{
		std::list<PrologBindings>::iterator it = data_;
		if(++it != query_->bindings_.end())
		{
		    ++data_;
		    return;
		}
	}
	if(query_->finished_)
	{
		data_ = query_->bindings_.end();
		return;
	}
	if(requestNextSolution())
	{
		++data_;
	}
}

bool PrologQuery::iterator::requestNextSolution()
{
	json_prolog_msgs::PrologNextSolutionRequest req;
	json_prolog_msgs::PrologNextSolutionResponse resp;
	
	req.id = query_->query_id_;
	if(!query_->prolog_->next_solution.call(req, resp))
	    throw PrologQuery::QueryError("Service call failed.");
	
	switch(resp.status){
	case json_prolog_msgs::PrologNextSolutionResponse::NO_SOLUTION:
		data_ = query_->bindings_.end();
			query_->finished_ = true;
		return false;
	case json_prolog_msgs::PrologNextSolutionResponse::WRONG_ID:
		query_->finished_ = true;
		throw PrologQuery::QueryError(
		    "Wrong id. Maybe the server is already processing a query.");
	case json_prolog_msgs::PrologNextSolutionResponse::QUERY_FAILED:
		query_->finished_ = true;      
		throw PrologQuery::QueryError(
		    "Prolog query failed: " + resp.solution);
	case json_prolog_msgs::PrologNextSolutionResponse::OK:
		query_->bindings_.push_back(PrologBindings::parseJSONBindings(resp.solution));
		return true;
	default:
		query_->finished_ = true;      
		throw PrologQuery::QueryError("Unknow query status.");
	}
}

bool PrologQuery::iterator::equal(const iterator &other) const
{
	if(!other.query_ && query_ && data_ == query_->bindings_.end())
		return true;
	else if(!query_ && other.query_ && other.data_ == other.query_->bindings_.end())
		return true;
	else
		return (!query_ && !other.query_) || (data_ == other.data_);
}

PrologQuery::PrologQuery(PrologClient &prolog, const std::string &query_str)
: finished_(false),
  prolog_(&prolog),
  query_id_(makeQueryId())
{
	json_prolog_msgs::PrologQueryRequest req;
	json_prolog_msgs::PrologQueryResponse resp;

	req.id = query_id_;
	req.query = query_str;

	if(!prolog_->prolog_query.isValid() || !prolog_->prolog_query.exists())
	{
		throw ServerNotFound("No connection to the rosprolog server.");
	}
	if(!prolog_->prolog_query.call(req, resp))
	{
		throw PrologQuery::QueryError("Service call '" + prolog_->prolog_query.getService() + "' failed");
	}
	if(!resp.ok)
	{
		throw PrologQuery::QueryError("Prolog query failed: " + resp.message);
	}
	// Instantiate the first solution
	PrologQuery::iterator(*this).requestNextSolution();
}

PrologQuery::iterator PrologQuery::begin()
{
	return PrologQuery::iterator(*this);
}

void PrologQuery::finish()
{
	json_prolog_msgs::PrologFinishRequest req;
	json_prolog_msgs::PrologFinishResponse resp;
	
	req.id = query_id_;
	if(!prolog_->prolog_finish.call(req, resp))
		throw PrologQuery::QueryError("Service call '" + prolog_->prolog_finish.getService() + "' failed");
	
	finished_ = true;
}

std::string PrologQuery::makeQueryId()
{
	static int counter = 0;
	return "ROSPROLOG_CPP_" + boost::lexical_cast<std::string>(ros::Time::now().toNSec()) +
		boost::lexical_cast<std::string>(counter++);
}
