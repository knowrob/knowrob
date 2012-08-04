package edu.tum.cs.ias.knowrob.comp_barcoo;

import ros.*;
import ros.pkg.vision_msgs.msg.aposteriori_position;
import ros.pkg.vision_msgs.msg.cop_answer;
import ros.pkg.vision_msgs.msg.cop_descriptor;
import ros.pkg.vision_srvs.srv.cop_call;
import ros.pkg.vision_srvs.srv.srvjlo;
import ros.pkg.vision_srvs.srv.srvjlo.Response;
import ros.pkg.vision_srvs.srv.srvjlo.Request;
import ros.pkg.vision_msgs.msg.partial_lo;

import java.util.concurrent.ExecutionException;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Queue;
import java.util.Vector;
import java.lang.Exception;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.lang.Runnable;

import jpl.Query;

public class BarcooLocationInferer {
	
	boolean initialized;
	
	public BarcooLocationInferer()
	{
		initialized = false;
		final Ros ros = Ros.getInstance();		
		ros.init("Barcoo Location Inference");				
		NodeHandle n = ros.createNodeHandle();
		
		ServiceServer.Callback<infer_location.Request,infer_location.Response> scb = new ServiceServer.Callback<infer_location.Request,infer_location.Response>() 
		{
            public infer_location.Response call(infer_location.Request request) {
            	infer_location.Response res = new send_barcode.Response();
            	if(!initialized)
            	{            		
            		BarcooLocationInferer.executeQuery("register_ros_package(mod_vis)");
            		BarcooLocationInferer.executeQuery("use_module(library('mod_vis'))");
            		BarcooLocationInferer.executeQuery("mod_vis:visualisation_canvas(C)");
            		//wait until it is initialized
            	    sleep(5);
            		initialized = true;
            	}
            	String object = req.name_space+":'"+req.object_type+"'";
                String classifier = "best_location_maxMaxWup"; //best_location_dtree

                String query = "highlight_" + classifier + "(" + object + ", _)";
                String query2 = classifier + "(" + object + ", L)";
                ROS_INFO("Query highlight: %s", query.c_str());
                ROS_INFO("Query: %s", query2.c_str());
            	
                BarcooLocationInferer.executeQuery(query);
            	
            	
            }
		};   
		ServiceServer<infer_location.Request,infer_location.Response,infer_location> srv = n.advertiseService("Infer location", new infer_location(), scb);
		n.spin();
	}
	
	
	private static HashMap<String, Vector<Object>> executeQuery(String query) {
			
			HashMap<String, Vector<Object>> result = new HashMap< String, Vector<Object> >();
			Hashtable[] solutions;
	
			synchronized(jpl.Query.class) {
			
	    		Query q = new Query( "expand_goal(("+query+"),_9), call(_9)" );
	
	    		if(!q.hasSolution())
	    			return new HashMap<String, Vector<Object>>();
	    		
	    		
	    		solutions = q.allSolutions();
	    		for (Object key: solutions[0].keySet()) {
	    			result.put(key.toString(), new Vector<Object>());
	    		}
	    		
	    		// Build the result
	    		for (int i=0; i<solutions.length; i++) {
	    			Hashtable solution = solutions[i];
	    			for (Object key: solution.keySet()) {
	    				String keyStr = key.toString();
	    				if (!result.containsKey( keyStr )) {
	
	    					// previously unknown column, add result vector
	    					Vector<Object> resultVector = new Vector<Object>(); 
	    					resultVector.add( i, solution.get( key ).toString() );
	    					result.put(keyStr, resultVector);
	
	    				}
	    				// Put the solution into the correct vector
	    				Vector<Object> resultVector = result.get( keyStr );
	    				resultVector.add( i, solution.get( key ).toString() );
	    			}
	    		}
			}
			// Generate the final QueryResult and return
			return result;
	}
	public static void main(String[] args)
	{		
		try
		{
			BarcooLocationInferer inferer = new BarcooLocationInferer();			
			
			
		}
		catch(Exception ex)
		{
			ex.printStackTrace();
		}
	}

}
