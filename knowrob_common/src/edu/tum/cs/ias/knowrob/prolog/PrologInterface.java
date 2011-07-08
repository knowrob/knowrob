package edu.tum.cs.ias.knowrob.prolog;

import java.util.HashMap;
import java.util.Hashtable;
import java.util.Vector;
import jpl.*;



public class PrologInterface {

	/**
	 * Wrapper around the JPL Prolog interface
	 * 
	 * @param query A query string in common SWI Prolog syntax
	 * @return A HashMap<VariableName, ResultsVector>
	 */
	public static HashMap<String, Vector<String>> executeQuery(String query) {
		
		HashMap<String, Vector<String>> result = new HashMap< String, Vector<String> >();
		Hashtable[] solutions;

		synchronized(jpl.Query.class) {
		
    		Query q = new Query( "expand_goal(("+query+"),_9), call(_9)" );

    		if(!q.hasSolution())
    			return new HashMap<String, Vector<String>>();
    		
    		
    		solutions = q.allSolutions();
    		for (Object key: solutions[0].keySet()) {
    			result.put(key.toString(), new Vector<String>());
    		}
    		
    		// Build the result
    		for (int i=0; i<solutions.length; i++) {
    			Hashtable solution = solutions[i];
    			for (Object key: solution.keySet()) {
    				String keyStr = key.toString();
    				if (!result.containsKey( keyStr )) {

    					// previously unknown column, add result vector
    					Vector<String> resultVector = new Vector<String>(); 
    					resultVector.add( i, solution.get( key ).toString() );
    					result.put(keyStr, resultVector);

    				}
    				// Put the solution into the correct vector
    				Vector<String> resultVector = result.get( keyStr );
    				resultVector.add( i, solution.get( key ).toString() );
    			}
    		}
		}
		// Generate the final QueryResult and return
		return result;
	}
	
}
