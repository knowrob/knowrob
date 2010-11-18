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

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Iterator;
import java.util.NoSuchElementException;

import ros.RosException;
import ros.pkg.json_prolog.srv.PrologFinish;
import ros.pkg.json_prolog.srv.PrologNextSolution;
import ros.pkg.json_prolog.srv.PrologQuery;

/**
 * Wrapper class for Prolog queries that provides an iterator to step through the results.
 * @author Moritz Tenorth, tenorth@cs.tum.edu
 *
 */
public class PrologQueryProxy implements Iterable<PrologBindings> {

	boolean finished_;
	Prolog prolog_;
	String query_id_;
	ArrayList<PrologBindings> bindings_;
	QueryIterator q_it;
	
	
	/**
	 * Send a query to Prolog via the json_prolog interface and provide
	 * methods to iterate over the result set.
	 * 
	 * @param prolog Instance of the Prolog client class
	 * @param query_str Query as Prolog query string
	 */
	public PrologQueryProxy(Prolog prolog, String query_str) {

		this.query_id_ = makeQueryID();
		this.prolog_ = prolog;
		this.finished_ = false;
		this.bindings_ = new ArrayList<PrologBindings>();
		
		PrologQuery.Request req = new PrologQuery.Request();
		PrologQuery.Response resp = new PrologQuery.Response();

		req.id = query_id_;
		req.query = query_str;

		
		try{
			
			resp=prolog_.prolog_query.call(req);
			
		} catch (RosException e) {
			throw(new QueryError("Service call '" + prolog_.prolog_query.getService() + "' failed"));
		}
		
		if(resp.ok == 0) {
			throw(new QueryError("Prolog query failed: " + resp.message));
		}

		
		// Instantiate the first solution
		q_it = new QueryIterator(this);
		//q_it.requestNextSolution();

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

		PrologFinish.Request req = new PrologFinish.Request();

		req.id = query_id_;		
		try{
			
			prolog_.prolog_finish.call(req);
			
		} catch (RosException e) {
			throw(new QueryError("Service call '" + prolog_.prolog_finish.getService() + "' failed"));
		}
		finished_ = true;
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
			
			  PrologNextSolution.Request req = new PrologNextSolution.Request();
			  PrologNextSolution.Response resp = new PrologNextSolution.Response();

			  req.id = query_.query_id_;
			  
				try{
					
					resp=prolog_.next_solution.call(req);
					
				} catch (RosException e) {
					throw(new QueryError("Service call '" + prolog_.next_solution.getService() + "' failed"));
				}

			  switch(resp.status) {
			  
			    case PrologNextSolution.Response.NO_SOLUTION:
			    	finish();
			    	data_ = query_.bindings_.listIterator(query_.bindings_.size());
			    	query_.finished_ = true;
			    	return false;
			      
			    case PrologNextSolution.Response.WRONG_ID:
			    	finish();
			    	query_.finished_ = true;
			    	throw(new PrologQueryProxy.QueryError("Wrong id. Maybe the server is already processing a query."));
			      
			    case PrologNextSolution.Response.QUERY_FAILED:
			    	finish();
			    	query_.finished_ = true;      
			    	throw(new PrologQueryProxy.QueryError("Prolog query failed: " + resp.solution));
			      
			    case PrologNextSolution.Response.OK:
			    	query_.bindings_.add(PrologBindings.parseJSONBindings(resp.solution));
			    	return true;
			      
			    default:
			    	finish();
			    	query_.finished_ = true;      
			    	throw(new PrologQueryProxy.QueryError("Unknow query status."));
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
