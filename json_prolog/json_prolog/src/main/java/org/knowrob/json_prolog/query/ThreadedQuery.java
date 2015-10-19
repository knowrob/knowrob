/* 
 * Copyright (c) 2014, Daniel Beßler
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

package org.knowrob.json_prolog.query;

import java.util.Hashtable;
import java.util.LinkedList;

import jpl.Term;

/**
 * A prolog query that is processed in a separate thread.
 * Each query has a prolog engine assigned and can only be
 * processed exclusively by one thread.
 * Incremental queries require the ThreadedQuery in order to make
 * sure that the query is processed in one thread only.
 * 
 * @author Daniel Beßler
 */
public class ThreadedQuery implements Runnable {

	private String queryString = null;

	private Term queryTerm = null;
	
	private jpl.Query query = null;
	
	private boolean isStarted = false;
	
	private boolean isRunning = true;
	
	private boolean isClosed = true;
	
	private LinkedList<QueryCommand> commadQueue = new LinkedList<QueryCommand>();

	public ThreadedQuery(String queryString) {
		this.queryString = queryString;
	}
	
	public ThreadedQuery(Term term) {
		this.queryTerm = term;
	}

	@Override
	public void run() {
		isStarted = true;
		isClosed = false;
		
		// Create a query (bound to this thread)
		if(queryString!=null) {
			query = new jpl.Query(queryString);
		}
		else if(queryTerm!=null) {
			query = new jpl.Query(queryTerm);
		}
		else {
			query = null;
			return;
		}
		
		QueryCommand cmd = null;
		
		try {
			while(isRunning) {
				if(commadQueue.isEmpty()) {
					// Wait for command to be pushed
					synchronized (this) {
						try {
							this.wait();
						}
						catch (Exception e) {}
					}
				}
				else {
					synchronized (commadQueue) {
						cmd = commadQueue.poll();
					}
					cmd.result = cmd.execute(query);
					if(cmd.result == null) {
						cmd.result = new String("Result is null.");
					}
				}
			}
		}
		catch(Exception exc) {
			exc.printStackTrace();
			// Notify caller that command finished
			for(QueryCommand x : commadQueue) {
				x.result = exc;
			}
			if(cmd != null) {
				cmd.result = exc;
			}
		}

		isClosed = true;
		isRunning = false;
		query.close();
	}

	public void close() {
		if(!isClosed && isRunning) {
			isRunning = false;
			// Notify caller that command finished
			for(QueryCommand cmd : commadQueue) {
				if(cmd.result==null)
					cmd.result = new String("Query was closed.");
			}
			// wake up query thread
			synchronized (this) { this.notifyAll(); }
		}
	}

	private Object runCommand(QueryCommand cmd) throws Exception {
		if(isClosed || !isRunning) {
		  throw new InterruptedException("Thread not running for query.");
		}
		
		cmd.result = null;
		synchronized (commadQueue) { commadQueue.push(cmd); }
		// wake up query thread
		synchronized (this) { this.notifyAll(); }
		
		while(cmd.result==null) {
			try {
				Thread.sleep(1);
			}
			catch (InterruptedException e) {}
		}
		if(cmd.result instanceof Exception)
		  throw (Exception)cmd.result;
		return cmd.result;
	}
	
	public boolean isRunning() {
		return isRunning;
	}

	public boolean isStarted() {
		return isStarted;
	}

	public boolean hasMoreSolutions() throws Exception {
		Object val = runCommand(new HasMoreSolutionsCommand());
		if(val instanceof Boolean) {
			return ((Boolean)val).booleanValue();
		}
		else {
			System.err.println(val.toString());
			return false;
		}
	}

	public void reset() throws Exception {
		runCommand(new ResetCommand());
	}

	@SuppressWarnings("unchecked")
	public Hashtable<String, Term> nextSolution() throws Exception {
		return (Hashtable<String, Term>)runCommand(new NextSolutionCommand());
	}

	@SuppressWarnings("unchecked")
	public Hashtable<String, Term>[] allSolutions() throws Exception {
		return (Hashtable<String, Term>[])runCommand(new AllSolutionsCommand());
	}
}
