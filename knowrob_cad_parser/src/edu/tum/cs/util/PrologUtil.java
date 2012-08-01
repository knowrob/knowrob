package edu.tum.cs.util;

import java.util.HashMap;
import java.util.Hashtable;
import java.util.Vector;

import jpl.Query;

/**
 * Static util class for communicating between prolog and java.
 * 
 * @author Stefan Profanter
 *
 */
public class PrologUtil {
	
	/**
	 * Execute the given query. Assert the given premises and retract them after querying.
	 * @param query the query.
	 * @return the HashMap representing the bindings of the variables.
	 * 
	 *	TODO Merge with knowrob_common PrologInterface
	 */
	@SuppressWarnings("rawtypes")
	public static HashMap<String, Vector<Object>> executeQuery(String query) {
		
		HashMap<String, Vector<Object>> result = new HashMap< String, Vector<Object> >();
		Hashtable[] solutions;
		
		Query q = new Query( "expand_goal(("+query+"),_9), call(_9)" );
			// Due to bugs we have to check for one answer beforehand.
			if (!q.hasMoreSolutions())
				return new HashMap<String, Vector<Object>>();
			Hashtable oneSolution = q.nextSolution();
			if (oneSolution.isEmpty())	// Due to a bug consulting a file without anything else results in shutdown
				return new HashMap<String, Vector<Object>>();	// I will try to prevent it with this construction
			
		// Restart the query and fetch everything.
		q.rewind();
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
		// Generate the final QueryResult and return
		return result;
	}
}
