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

import java.util.Map;
import java.util.LinkedList;

import org.jpl7.Term;

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

	private org.jpl7.Term queryTerm = null;
	
	private org.jpl7.Query query = null;
	
	private boolean isStarted = false;
	
	private boolean isRunning = true;
	
	private boolean isClosed = true;
	
	private LinkedList<QueryCommand> commadQueue = new LinkedList<QueryCommand>();

	private QueryCommand currentCommand = null;

	private Exception exception = null;

	public ThreadedQuery(String queryString) {
		this.queryString = queryString;
	}
	
	public ThreadedQuery(org.jpl7.Term term) {
		this.queryTerm = term;
	}

	public Object getQueryObject() {
		return queryString!=null ? queryString : queryTerm;
	}
	
	public void waitOnThread() {
		  while(!isStarted()) {
				synchronized (getQueryObject()) {
					try { getQueryObject().wait(); }
					catch (Exception e) {}
				}
		  }
	}

	@Override
	public void run() {
		isStarted = true;
		isClosed = false;
		
		// Create a query (bound to this thread)
		try {
			if(queryString!=null) {
				query = new org.jpl7.Query(queryString);
			}
			else if(queryTerm!=null) {
				query = new org.jpl7.Query(queryTerm);
			}
			else {
				throw new RuntimeException("No query defined!");
			}
		}
		catch(Exception exc) {
			query = null;
			isClosed = true;
			isRunning = false;
			exception  = exc;
			// Wake up caller waiting on the thread to be started
			synchronized (getQueryObject()) {
				try { getQueryObject().notifyAll(); }
				catch (Exception e) {}
			}
			return;
		}
		// Wake up caller waiting on the thread to be started
		// (i.e., wake up caller of the method `waitOnThread`)
		synchronized (getQueryObject()) {
			try { getQueryObject().notifyAll(); }
			catch (Exception e) {}
		}
		// Start processing query commands
		QueryCommand cmd = null;
		try {
			while(isRunning) { // until thread is closed
				if(commadQueue.isEmpty()) {
					// Wait for command to be pushed onto the command queue
					synchronized (this) {
						try { this.wait(); }
						catch (Exception e) {}
					}
				}
				else {
					// poll command from queue
					synchronized (commadQueue) {
						cmd = commadQueue.poll();
					}
					currentCommand = cmd;
					// process the command
					cmd.result = cmd.execute(query);
					currentCommand = null;
					// ensure cmd.result is not null
					if(cmd.result == null) {
						cmd.result = new QueryYieldsNullException(query);
					}
					synchronized(cmd) { cmd.notifyAll(); }
				}
			}
		}
		catch(Exception exc) {
			// Notify caller that command finished
			for(QueryCommand x : commadQueue) {
				x.result = exc;
				synchronized(x) { x.notifyAll(); }
			}
			if(cmd != null) {
				cmd.result = exc;
				synchronized(cmd) { cmd.notifyAll(); }
			}
		}

		isClosed = true;
		isRunning = false;
		query.close();
	}

	public void close() {
		if(!isClosed && isRunning) {
			isRunning = false;
			// Notify caller that command finished (e.g., in case query was closed when a command
			// did not completed yet)
			for(QueryCommand cmd : commadQueue) {
				if(cmd.result==null) cmd.result = new QueryClosedException(query);
				synchronized(cmd) { cmd.notifyAll(); }
			}
			// FIXME: what happens if thread is stuck with a command that does not terminate?
			if(currentCommand!=null) {
				currentCommand.result = new QueryClosedException(query);
				synchronized(currentCommand) { currentCommand.notifyAll(); }
				currentCommand = null;
			}
			// wake up query thread so that it can terminate after close was called
			synchronized (this) { this.notifyAll(); }
		}
	}

	private Object runCommand(QueryCommand cmd) throws Exception {
		waitOnThread();
		if(exception!= null) {
			throw exception;
		}
		if(isClosed || !isRunning) {
		  throw new InterruptedException("Thread not running for query.");
		}
		
		cmd.result = null;
		// add command to queue which is processed in query thread
		synchronized (commadQueue) { commadQueue.push(cmd); }
		// wake up query thread in case it is sleeping
		synchronized (this) { this.notifyAll(); }
		// wait until query thread processed the command
		if(cmd.result==null) {
			synchronized(cmd) {
				try { cmd.wait(); }
				catch (Exception e) {}
			}
		}
		// handle query result. in case it's an exception, throw it!
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

	public void reset() throws Exception {
		runCommand(new ResetCommand());
	}

	public boolean hasMoreSolutions() throws Exception {
		return ((Boolean)runCommand(new HasMoreSolutionsCommand())).booleanValue();
	}

	@SuppressWarnings("unchecked")
	public java.util.Map<String, Term> nextSolution() throws Exception {
		return (java.util.Map<String, Term>)runCommand(new NextSolutionCommand());
	}

	@SuppressWarnings("unchecked")
	public Map<String, Term>[] allSolutions() throws Exception {
		return (Map<String, Term>[])runCommand(new AllSolutionsCommand());
	}
}
