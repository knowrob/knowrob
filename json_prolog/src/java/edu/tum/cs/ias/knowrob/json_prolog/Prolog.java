/* 
 * Copyright (c) 2010, Moritz Tenorth
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

package edu.tum.cs.ias.knowrob.json_prolog;

import ros.NodeHandle;
import ros.Ros;
import ros.ServiceClient;

import ros.pkg.json_prolog.srv.PrologQuery;
import ros.pkg.json_prolog.srv.PrologFinish;
import ros.pkg.json_prolog.srv.PrologNextSolution;


/**
 * Client wrapper library for the json_prolog interface
 * 
 * Main class to be used in other applications. For example usage, see {@code JSONPrologTestClient}
 * 
 * The implementation was intentionally kept similar to the C++ client library by Lorenz M\"osenlechner
 * 
 * @author Moritz Tenorth
 *
 */
public class Prolog {

	private static Ros ros;
	private static NodeHandle n;
	
	protected static void initRos() {

    	ros = Ros.getInstance();

		if(!Ros.getInstance().isInitialized()) {
	    	ros.init("json_prolog_java_client");
		}
		n = ros.createNodeHandle();

	}
	
	public ServiceClient<PrologQuery.Request, PrologQuery.Response, PrologQuery> prolog_query;
	public ServiceClient<PrologNextSolution.Request, PrologNextSolution.Response, PrologNextSolution> next_solution;
	public ServiceClient<PrologFinish.Request, PrologFinish.Response, PrologFinish> prolog_finish;

	
	
	/**
	 * Constructor. Specify the ROS name space (default: "/json_prolog")
	 * 
	 * @param ns
	 */
	public Prolog(String ns) {
		
		initRos();
		prolog_query = n.serviceClient(ns + "/simple_query", new PrologQuery());
		next_solution = n.serviceClient(ns + "/next_solution", new PrologNextSolution());
		prolog_finish = n.serviceClient(ns + "/finish", new PrologFinish());				
	}
	public Prolog() {
		this("/json_prolog");
	}
	
	/**
	 * Send a query to Prolog
	 * 
	 * @param query_str The query to be sent to Prolog (standard SWI Prolog syntax)
	 * @return An instance of the PrologQueryProxy that can be used to iterate over the results
	 * 
	 */
	public PrologQueryProxy query(String query_str) {
		return new PrologQueryProxy(this, query_str);
	}
	
	/**
	 * Send a query to Prolog and retrieve only the first result. Finishes the
	 * query after retrieving the result.
	 * 
	 * @param query_str The query to be sent to Prolog (standard SWI Prolog syntax)
	 * @return PrologBindings for the first result.
	 * 
	 */
	public PrologBindings once(String query_str) {
		
		PrologQueryProxy query = new PrologQueryProxy(this, query_str);
		PrologBindings result = query.iterator().next();
		query.finish();
		return result;
	}
	 

	
}
