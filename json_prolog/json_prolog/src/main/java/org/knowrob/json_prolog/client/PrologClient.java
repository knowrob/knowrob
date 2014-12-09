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

package org.knowrob.json_prolog.client;

import org.knowrob.json_prolog.PrologBindings;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;


/**
 * Client wrapper library for the json_prolog interface
 * 
 * Main class to be used in other applications. For example usage, see {@code JSONPrologTestClient}
 * 
 * @author Moritz Tenorth
 *
 */
public class PrologClient extends AbstractNodeMain {

	public ServiceClient<json_prolog_msgs.PrologQueryRequest, json_prolog_msgs.PrologQueryResponse> query_client;
	public ServiceClient<json_prolog_msgs.PrologNextSolutionRequest, json_prolog_msgs.PrologNextSolutionResponse> next_solution_client;
	public ServiceClient<json_prolog_msgs.PrologFinishRequest, json_prolog_msgs.PrologFinishResponse> finish_client;
	
	public ConnectedNode node;

	
	
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		
		this.node = connectedNode;
		
		// wait for node to be ready
		try {
			while(node == null) {
				Thread.sleep(200);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		try {
			query_client = connectedNode.newServiceClient("json_prolog/simple_query", json_prolog_msgs.PrologQuery._TYPE);
		} catch (ServiceNotFoundException e) {
			throw new RosRuntimeException(e);
		}
		
		try {
			next_solution_client = connectedNode.newServiceClient("json_prolog/next_solution", json_prolog_msgs.PrologNextSolution._TYPE);
		} catch (ServiceNotFoundException e) {
			throw new RosRuntimeException(e);
		}
		
		try {
			finish_client = connectedNode.newServiceClient("json_prolog/finish", json_prolog_msgs.PrologFinish._TYPE);
		} catch (ServiceNotFoundException e) {
			throw new RosRuntimeException(e);
		} 
	}
	
	
	/**
	 * Send a query to Prolog
	 * 
	 * @param query_str The query to be sent to Prolog (standard SWI Prolog syntax)
	 * @return An instance of the PrologQueryProxy that can be used to iterate over the results
	 * 
	 */
	public PrologQueryProxy query(String query_str) {
		
		// wait for node to be ready
		try {
			while(node == null) {
				Thread.sleep(200);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
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
		
		// wait for node to be ready
		try {
			while(node == null) {
				Thread.sleep(200);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		PrologQueryProxy query = new PrologQueryProxy(this, query_str);
		PrologBindings result = query.iterator().next();
		query.finish();
		return result;
	}
	 

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("json_prolog_client");
	}
}
