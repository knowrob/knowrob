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

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Iterator;
import java.util.NoSuchElementException;

import org.knowrob.json_prolog.PrologBindings;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.node.service.ServiceResponseListener;

/**
 * Wrapper class for Prolog queries that provides an iterator to step through the results.
 * @author Moritz Tenorth, tenorth@cs.tum.edu
 *
 */
public class PrologQueryProxy implements Iterable<PrologBindings> {

	boolean finished_;
	PrologClient prolog_;
	String query_id_;
	ArrayList<PrologBindings> bindings_;
	QueryIterator q_it;
	
	json_prolog_msgs.PrologNextSolutionResponse next_sol_res = null;
	
	
	
	/**
	 * Send a query to Prolog via the json_prolog interface and provide
	 * methods to iterate over the result set.
	 * 
	 * @param prolog Instance of the Prolog client class
	 * @param query_str Query as Prolog query string
	 */
	public PrologQueryProxy(final PrologClient prolog, String query_str) {

		this.query_id_ = makeQueryID();
		this.prolog_ = prolog;
		this.finished_ = false;
		this.bindings_ = new ArrayList<PrologBindings>();

		
		// wait for node to be ready
		try {
			while(prolog.query_client == null) {
				Thread.sleep(200);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		final json_prolog_msgs.PrologQueryRequest req = prolog.query_client.newMessage();
		req.setId(query_id_);
		req.setQuery(query_str);

		prolog.query_client.call(req, new ServiceResponseListener<json_prolog_msgs.PrologQueryResponse>() {

			@Override
			public void onSuccess(json_prolog_msgs.PrologQueryResponse response) {
				prolog_.node.getLog().debug("Query " + req.getQuery() + "successful");
			}

			@Override
			public void onFailure(RemoteException e) {
				throw new RosRuntimeException(e);
			}
		});

		// Instantiate the first solution
		q_it = new QueryIterator(this);
	}
	
	
	
	/**
	 * Iterator for the result set of the query
	 */
	public Iterator<PrologBindings> iterator() {
		return q_it;
	}
	

	/**
	 * Check if the Prolog query is finished
	 */
	public boolean isFinished()	{
		return finished_;
	}
	
	
	/**
	 * Finish the Prolog query (i.e. close it).
	 */
	public void finish()	{

		final json_prolog_msgs.PrologFinishRequest req = prolog_.finish_client.newMessage();
		req.setId(query_id_);

		prolog_.finish_client.call(req, new ServiceResponseListener<json_prolog_msgs.PrologFinishResponse>() {

			@Override
			public void onSuccess(json_prolog_msgs.PrologFinishResponse response) {
				prolog_.node.getLog().debug("Query successfully finished");
				finished_ = true;
			}

			@Override
			public void onFailure(RemoteException e) {
				throw new RosRuntimeException(e);
			}
		});
	}

	
	/**
	 * Implementation of an iterator for a lazy list
	 * for the query results. Retrieves results from 
	 * json_prolog if possible when hasNext() is called.
	 * 
	 * @author Moritz Tenorth, tenorth@cs.tum.edu
	 *
	 */
	class QueryIterator implements Iterator<PrologBindings> {

		Iterator<PrologBindings> data_;
	    PrologQueryProxy query_;
	    
	    
		public QueryIterator(PrologQueryProxy proxy) {
			query_ = proxy;
			data_=proxy.bindings_.iterator(); // initialize iterator to the first element in the bindings list
		}

		
		/**
		 * Check if there are any more results; retrieves new
		 * results from json_prolog if available.
		 */
		@Override
		public boolean hasNext() {
			return requestNextSolution();
		}

		
		/**
		 * Return the next element in the list. hasNext makes sure
		 * there is a next elment.
		 */
		@Override
		public PrologBindings next() {
			
			if(query_==null)
				throw( new NullPointerException());
			
			// just return the next element if there are still some in bindings_
			if(data_.hasNext()) {
				data_ = query_.bindings_.listIterator(query_.bindings_.size()-1);
				return data_.next();
			}
  
			// return last element if the query is finished
			if(query_.finished_) {
				data_ = query_.bindings_.listIterator(query_.bindings_.size()-1);
				return data_.next();
			}

			else throw( new NoSuchElementException());
		}

		
		@Override
		public void remove() {
			// do nothing: we do not remove elements from the result set
			return;
		}
	
	
		/**
		 * Check if there is a next solution, either already in the list data_
		 * or available via json_prolog
		 * 
		 * @return True if there is a next solution (which is being added to the data_ buffer)
		 */
		protected boolean requestNextSolution() {
			
			// just return the next element if there are still some in bindings_
			if(data_.hasNext()) {
				return true;
			}
			
			
			final json_prolog_msgs.PrologNextSolutionRequest req = prolog_.next_solution_client.newMessage();
			req.setId(query_.query_id_);
			next_sol_res = null;
			
			prolog_.next_solution_client.call(req, new ServiceResponseListener<json_prolog_msgs.PrologNextSolutionResponse>() {

				@Override
				public void onSuccess(json_prolog_msgs.PrologNextSolutionResponse resp) {
					synchronized(prolog_.next_solution_client) {
						next_sol_res = resp;
						prolog_.next_solution_client.notifyAll();
					}
				}

				@Override
				public void onFailure(RemoteException e) {
					throw new RosRuntimeException(e);
				}
			});
			
			synchronized(prolog_.next_solution_client) {
				
				try {
					
					if(next_sol_res==null) {
						prolog_.next_solution_client.wait(4000);
					}
					
					
					switch(next_sol_res.getStatus()) {

						case json_prolog_msgs.PrologNextSolutionResponse.NO_SOLUTION:
							finish();
							data_ = query_.bindings_.listIterator(query_.bindings_.size());
							query_.finished_ = true;
							return false;
	
						case json_prolog_msgs.PrologNextSolutionResponse.WRONG_ID:
							finish();
							query_.finished_ = true;
							throw(new PrologQueryProxy.QueryError("Wrong id. Maybe the server is already processing a query."));
	
						case json_prolog_msgs.PrologNextSolutionResponse.QUERY_FAILED:
							finish();
							query_.finished_ = true;
							throw(new PrologQueryProxy.QueryError("Prolog query failed: " + next_sol_res.getSolution()));
	
						case json_prolog_msgs.PrologNextSolutionResponse.OK:
							query_.bindings_.add(PrologBindings.parseJSONBindings(next_sol_res.getSolution()));
							return true;
	
						default:
							finish();
							query_.finished_ = true;
							throw(new PrologQueryProxy.QueryError("Unknow query status."));
					}
				} catch(InterruptedException e) {
					e.printStackTrace();
					return false;
				}
			}

		}
	};


	/**
	 * Create a quasi-unique ID for each query 
	 * @return The query ID
	 */
	protected String makeQueryID() {
 		SimpleDateFormat sdf = new SimpleDateFormat("yy-MM-dd_HH-mm-ss-SSS");
 		return "JAVA_QUERY_"+sdf.format(new Date());
	}
	
	
	/**
	 * Exception class for query errors
	 *
	 */
	class QueryError extends RuntimeException {
		private static final long serialVersionUID = 1190356215893161319L;
		public QueryError(String msg) {
			super(msg);
		}
	};

}
