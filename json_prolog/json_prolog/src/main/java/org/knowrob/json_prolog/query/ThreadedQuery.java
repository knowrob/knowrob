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

	private static final long IDLE_SLEEP = 100;

	private String queryString = null;

	private Term queryTerm = null;
	
	private jpl.Query query = null;
	
	private boolean isRunning = false;
	
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
		isRunning = true;
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
		
		QueryCommand cmd;
		
		try {
			while(isRunning) {
				if(commadQueue.isEmpty()) {
					synchronized (this) {
						try {
							// Wait for command to be pushed
							this.wait();
						}
						catch (Exception e) {}
					}
				}
				else {
					synchronized (this) {
						cmd = commadQueue.poll();
					}
					cmd.result = cmd.execute(query);
					synchronized (cmd) {
						// Notify caller that command finished
						cmd.notifyAll();
					}
				}
			}
		}
		catch(Exception exc) {
			// TODO: pass exception to ROS service
		}
		
		query.close();
		isClosed = true;
		isRunning = false;
	}

	public void close() {
		if(!isClosed && isRunning) {
			isRunning = false;
			synchronized (this) {
				// wake up query thread
				this.notifyAll();
			}
			// Wait until thread closed
			while(!isClosed) {
				try {
					Thread.sleep(IDLE_SLEEP);
				}
				catch (Exception e) {}
			}
		}
	}

	private Object runCommand(QueryCommand cmd) {
		synchronized (this) {
			commadQueue.push(cmd);
		}
		if(commadQueue.size()==1) {
			synchronized (this) {
				// wake up query thread
				this.notifyAll();
			}
		}
		synchronized (cmd) {
			try {
				// Wait until query completed
				cmd.wait();
			}
			catch (InterruptedException e) {}
		}
		return cmd.result;
	}

	public boolean hasMoreSolutions() {
		return ((Boolean)runCommand(new HasMoreSolutionsCommand())).booleanValue();
	}

	public void reset() {
		runCommand(new ResetCommand());
	}

	@SuppressWarnings("unchecked")
	public Hashtable<String, Term> nextSolution() {
		return (Hashtable<String, Term>)runCommand(new NextSolutionCommand());
	}

	@SuppressWarnings("unchecked")
	public Hashtable<String, Term>[] allSolutions() {
		return (Hashtable<String, Term>[])runCommand(new AllSolutionsCommand());
	}
}
