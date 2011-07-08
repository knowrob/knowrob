/* 
 * Copyright (c) 2010, Lorenz Moesenlechner
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

import java.util.*;
import java.io.*;

import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.ServiceServer;
import ros.pkg.json_prolog.srv.PrologFinish;
import ros.pkg.json_prolog.srv.PrologNextSolution;
import ros.pkg.json_prolog.srv.PrologQuery;


/**
 * ROS service interface to rosprolog
 * 
 * @author Lorenz Moesenlechner
 *
 */
public final class JSONPrologNode {
  private Hashtable<String, PrologSolutions> queries;
  private boolean hasIncrementalQuery = false; 
  private String initPackage="";

  public JSONPrologNode() {
    this("");
  }

  public JSONPrologNode(String initpkg) {
    this.initPackage=initpkg;
    queries = new Hashtable<String, PrologSolutions>();
  }


  static private class RospackError extends Exception {
    private static final long serialVersionUID = 1L;
  }

  /**
   * Callback class to handle PrologQuery requests (i.e. those creating a query that consists of terms etc)
   * 
   * @author Lorenz Moesenlechner
   *
   */
  private class QueryCallback implements ServiceServer.Callback<PrologQuery.Request, PrologQuery.Response> {


    @Override
    public PrologQuery.Response call(PrologQuery.Request request) {

      PrologQuery.Response response = new PrologQuery.Response();

      if (hasIncrementalQuery ) {
        response.ok = false;
        response.message = "Already processing an incremental query.";

      } else if ( queries.get(request.id) != null ) {
        response.ok = false;
        response.message = "Already processing a query with id " + request.id;

      } else {
        try {
          Ros.getInstance().logDebug("Received query with id " + request.id);

          synchronized(jpl.Query.class) {

            jpl.Query currentQuery = JSONQuery.makeQuery(request.query);
            String currentQueryId = request.id;

            if(request.mode == PrologQuery.Request.INCREMENTAL) {
              queries.put(currentQueryId, new PrologIncrementalSolutions(currentQuery));
              hasIncrementalQuery = true;
            }

            else {
              queries.put(currentQueryId, new PrologAllSolutions(currentQuery));
            }
          }
          response.ok = true;

        } catch (JSONQuery.InvalidJSONQuery e) {
          response.ok = false;
          response.message = e.toString();

        } catch (jpl.JPLException e) {
          response.ok = false;
          response.message = e.getMessage();
        }
      }
      return response;
    }
  }

  /**
   * Callback class to handle SimpleQuery requests (i.e. those sending a single string in Prolog syntax)
   * 
   * @author Lorenz Moesenlechner
   *
   */
  private class SimpleQueryCallback implements
  ServiceServer.Callback<PrologQuery.Request, PrologQuery.Response> {

    @Override
    public PrologQuery.Response call(PrologQuery.Request request) {


      PrologQuery.Response response = new PrologQuery.Response();

      try {

        synchronized(jpl.Query.class) {

          if (hasIncrementalQuery ) {
            response.ok = false;
            response.message = "Already processing an incremental query.";

          } else if (queries!=null && queries.get(request.id) != null ) {
            response.ok = false;
            response.message = "Already processing a query with id " + request.id;

          } else {
            try {
              Ros.getInstance().logDebug("Received query with id " + request.id);

              jpl.Query currentQuery = new jpl.Query(request.query);
              String currentQueryId = request.id;

              if(request.mode == PrologQuery.Request.INCREMENTAL) {
                queries.put(currentQueryId, new PrologIncrementalSolutions(currentQuery));
                hasIncrementalQuery = true;

              } else {
                queries.put(currentQueryId, new PrologAllSolutions(currentQuery));
              }
              response.ok = true;

            } catch (jpl.JPLException e) {
              response.ok = false;
              response.message = e.getMessage();
            }
          }
        }
      } catch (Exception e) {
        e.printStackTrace();
      }

      return response;
    }
  }


  /**
   * Callback for the NextSolution service
   * 
   * Depending on the query mode, this next solution is determined by retrieving the next solution
   * from Prolog or from an internal buffer of all solutions of the query.
   * 
   * @author Lorenz Moesenlechner
   *
   */
  private class NextSolutionCallback implements
  ServiceServer.Callback<PrologNextSolution.Request, PrologNextSolution.Response> {


    @Override
    public PrologNextSolution.Response call(PrologNextSolution.Request request) {

      PrologNextSolution.Response response = new PrologNextSolution.Response();
      try {
        synchronized(jpl.Query.class) {

          PrologSolutions currentQuery = queries.get(request.id);
          if (currentQuery == null)
            response.status = PrologNextSolution.Response.WRONG_ID;

          else if (!currentQuery.hasMoreSolutions()){
            response.status = PrologNextSolution.Response.NO_SOLUTION;
            removeQuery(request.id);

          } else {
            Hashtable<String, jpl.Term> solution = (Hashtable<String, jpl.Term>) currentQuery.nextSolution();
            response.solution = JSONQuery.encodeResult(solution).toString();
            response.status = PrologNextSolution.Response.OK;
          }
        }

      } catch (jpl.JPLException e) {
        response.solution = e.getMessage();
        response.status = PrologNextSolution.Response.QUERY_FAILED;
        removeQuery(request.id);

      } catch (Exception e) {
        e.printStackTrace();
      }

      return response;
    }
  }


  /**
   * Finish a query or all open queries if the request is '*' instead of a specific query ID.
   * 
   * @author Lorenz Moesenlechner
   *
   */
  private class FinishCallback implements 	
  ServiceServer.Callback<PrologFinish.Request, PrologFinish.Response> {

    @Override
    public PrologFinish.Response call(PrologFinish.Request request) {

      PrologFinish.Response response = new PrologFinish.Response();

      // finish all queries
      if (request.id.equals("*")){
        Enumeration<String> e = queries.keys();
        while(e.hasMoreElements())
          removeQuery(e.nextElement());
      } else {
        removeQuery(request.id);
      }
      return response;
    }
  }

  /**
   * Run the json_prolog node, i.e. launch the Prolog environment, 
   * initialize the ROS connection and register the callbacks 
   */
  public void execute(String args[]) throws InterruptedException, RosException, IOException, RospackError {

    // initialize the Prolog environment
    synchronized(jpl.Query.class) {

      initProlog();

      if(!this.initPackage.equals("")) {
        new jpl.Query("ensure_loaded('" + findRosPackage(initPackage)
            + "/prolog/init.pl')").oneSolution();
      }
    }

    // init ROS
    final Ros ros = Ros.getInstance();
    if(!Ros.getInstance().isInitialized()) {
      ros.init("json_prolog", false, false, false, args);
    }
    NodeHandle n = ros.createNodeHandle("~");

    // create services
    @SuppressWarnings("unused")
    ServiceServer<PrologQuery.Request, PrologQuery.Response, PrologQuery> query_srv = n
    .advertiseService("query", new PrologQuery(), new QueryCallback());
    @SuppressWarnings("unused")
    ServiceServer<PrologQuery.Request, PrologQuery.Response, PrologQuery> simple_query_srv = n
    .advertiseService("simple_query", new PrologQuery(), new SimpleQueryCallback());
    @SuppressWarnings("unused")
    ServiceServer<PrologNextSolution.Request, PrologNextSolution.Response, PrologNextSolution> next_solution_srv = n
    .advertiseService("next_solution", new PrologNextSolution(),
        new NextSolutionCallback());
    @SuppressWarnings("unused")
    ServiceServer<PrologFinish.Request, PrologFinish.Response, PrologFinish> finish_srv = n
    .advertiseService("finish", new PrologFinish(), new FinishCallback());


    ros.logInfo("json_prolog initialized and waiting for queries.");
    ros.spin();
  }


  /**
   * Remove a query (i.e. close it and reset the hasIncrementalQuery flag
   * @param id Query ID to be closed
   */
  public void removeQuery(String id) {

    PrologSolutions query = queries.get(id);

    if(query != null) {
      query.close();
      queries.remove(id);
      if(query instanceof PrologIncrementalSolutions)
        hasIncrementalQuery = false;
    }
  }


  /**
   * Initialize the SWI Prolog engine
   * 
   * @throws IOException
   * @throws InterruptedException
   * @throws RospackError
   */
  private static void initProlog() throws IOException, InterruptedException,
  RospackError {
    Vector<String> pl_args = new Vector<String>(Arrays.asList(jpl.JPL.getDefaultInitArgs()));
    pl_args.set(0, "/usr/bin/swipl");
    pl_args.add("-G256M");
    pl_args.add("-nosignals");
    jpl.JPL.setDefaultInitArgs(pl_args.toArray(new String[0]));
    jpl.JPL.init();
    new jpl.Query("ensure_loaded('" + findRosPackage("rosprolog") + "/prolog/init.pl')").oneSolution();
  }

  /**
   * Find a ROS package using the rospack program
   * 
   * @param name Name of the ROS package
   * @return Path to the ROS package
   * @throws IOException
   * @throws InterruptedException
   * @throws RospackError
   */
  private static String findRosPackage(String name) throws IOException, InterruptedException, RospackError {

    Process rospack = Runtime.getRuntime().exec("rospack find " + name);
    if (rospack.waitFor() != 0)
      throw new RospackError();
    return new BufferedReader(new InputStreamReader(rospack.getInputStream())).readLine();
  }



  public static void main(String args[]) {

    try {

      if(args.length>0)
        new JSONPrologNode(args[0]).execute(args);
      else
        new JSONPrologNode().execute(args);	

    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
