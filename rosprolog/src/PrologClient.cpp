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

#include <rosprolog/PrologClient.h>

#include <json_prolog_msgs/PrologQuery.h>
#include <json_prolog_msgs/PrologNextSolution.h>
#include <json_prolog_msgs/PrologFinish.h>

PrologClient::PrologClient(const std::string &ns) : nh_("~")
{
	prolog_query = nh_.serviceClient<json_prolog_msgs::PrologQuery>(ns + "/query");
	next_solution = nh_.serviceClient<json_prolog_msgs::PrologNextSolution>(ns + "/next_solution");
	prolog_finish = nh_.serviceClient<json_prolog_msgs::PrologFinish>(ns + "/finish");
}

PrologClient::PrologClient(const std::string &ns, bool persistent)
{
	prolog_query = nh_.serviceClient<json_prolog_msgs::PrologQuery>(ns + "/query", persistent);
	next_solution = nh_.serviceClient<json_prolog_msgs::PrologNextSolution>(ns + "/next_solution", persistent);
	prolog_finish = nh_.serviceClient<json_prolog_msgs::PrologFinish>(ns + "/finish", persistent);
}

PrologQuery PrologClient::query(const std::string &query_str)
{
	return PrologQuery(*this, query_str);
}

PrologBindings PrologClient::once(const std::string &query_str)
{
	PrologQuery query(PrologQuery(*this, query_str));
	PrologBindings result = *query.begin();
	query.finish();
	return result;
}

bool PrologClient::waitForServer(const ros::Duration &timeout)
{
	return prolog_query.waitForExistence(timeout) &&
	    next_solution.waitForExistence(timeout) &&
	    prolog_finish.waitForExistence(timeout);
}
