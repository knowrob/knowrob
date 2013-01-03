package edu.tum.cs.ias.knowrob.prolog;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Vector;

import com.google.common.base.Joiner;

import edu.tum.cs.ias.knowrob.owl.OWLThing;
import edu.tum.cs.ias.knowrob.utils.ros.RosUtilities;
import jpl.*;


public class PrologInterface {
    
	private static boolean initialized = false;


	/**
     *  Initialize a local Prolog engine with the ROS package given
     *  as argument
     *  
     *  @param initPackage String with the name of the package to be used for initialization
     *  
     */
    public static void initJPLProlog(String initPackage) {
    	
    	if(!initialized) {

    		try {
    			Vector<String> args= new Vector<String>(Arrays.asList(jpl.fli.Prolog.get_default_init_args()));
    			//            args.add( "-G256M" );
    			//args.add( "-q" );
    			args.add( "-nosignals" );

    			String rosprolog = RosUtilities.rospackFind("rosprolog");
    			jpl.fli.Prolog.set_default_init_args( args.toArray( new String[0] ) );

    			// load the appropriate startup file for this context
    			new jpl.Query("ensure_loaded('"+rosprolog+"/prolog/init.pl"+"')").oneSolution();
    			new jpl.Query("register_ros_package('"+initPackage+"')").oneSolution();

    			initialized = true;
    			
    		} catch(Exception e) {
    			e.printStackTrace();
    		}
    	}
    }
	
	
    /**
     *  Initialize a local Prolog engine with the init file given as argument
     *  
     *  @param initFile String with the path of the file to be used for initialization
     *  
     */
	public static void initLocalProlog(String initFile) {

		if(!PrologInterface.isInitialized()) {
			try {
				Vector<String> args= new Vector<String>(Arrays.asList(jpl.fli.Prolog.get_default_init_args()));
				// args.add( "-G256M" );
				// args.add( "-q" );
				args.add( "-nosignals" );
				jpl.fli.Prolog.set_default_init_args( args.toArray( new String[0] ) );

				// load the appropriate startup file for this context
				new jpl.Query("ensure_loaded('"+initFile+"')").oneSolution();

				PrologInterface.setInitialized(true);
				
			} catch(Exception e) {
				e.printStackTrace();
			}
		}
	}
    
	/**
	 * Wrapper around the JPL Prolog query interface. Assumes a Prolog engine to be
	 * initialized, e.g. using initJPLProlog
	 * 
	 * @param query A query string in common SWI Prolog syntax
	 * @return A HashMap<VariableName, ResultsVector>
	 */
    
	public static HashMap<String, Vector<String>> executeQuery(String query) {
		
		HashMap<String, Vector<String>> result = new HashMap< String, Vector<String> >();
		@SuppressWarnings("rawtypes")
		Hashtable[] solutions;

		synchronized(jpl.Query.class) {
		
    		Query q = new Query( "expand_goal(("+query+"),_9), call(_9)" );

    		if(!q.hasSolution())
    			return null;
    		
    		
    		solutions = q.allSolutions();
    		
    		if(solutions == null || solutions.length==0) // case: success, but no variable bindings
    			return result;
    		
    		for (Object key: solutions[0].keySet()) {
    			result.put(key.toString(), new Vector<String>());
    		}
    		
    		// Build the result
    		for (int i=0; i<solutions.length; i++) {
    			@SuppressWarnings("rawtypes")
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
	
	/**
	 * Convert Prolog list (dotted pairs) into a Java ArrayList
	 * 
	 * @param rest
	 * @return
	 */
    public static ArrayList<String[]> dottedPairsToArrayList(String rest) {
        
        ArrayList<String[]> bindings = new ArrayList<String[]>();
        while(rest.length()>0) {

            String[] l = rest.split("'\\.'", 2);
            if((l[0].equals("")) || (l[0].equals("("))) {
               rest=l[1]; continue;

            } else {
            	if (l[0].length() > 2)            	
            		bindings.add(new String[]{l[0].substring(1, l[0].length()-2).split(", ")[0]});
                                
                if(l.length>1) {
                    rest=l[1];  continue;
                } else break;
            }
            
        }
        return bindings;
      }

    
    /**
     * Removes single quotes at the start and end of a string if applicable. 
     * 
     * Useful when dealing with OWL IRI identifiers that need to be 
     * in single quotes in Prolog. 
     * 
     * @param str String with or without single quotes at the beginning and end
     * @return String without single quotes at the beginning and end
     */
    public static String removeSingleQuotes(String str) {
        if(str.startsWith("'"))
            str = str.substring(1);
        
        if(str.endsWith("'"))
            str = str.substring(0, str.length()-1);
        return str;
    }
    

    /**
     * Adds single quotes at the start and end of a string if applicable. 
     * 
     * Useful when dealing with OWL IRI identifiers that need to be 
     * in single quotes in Prolog. 
     * 
     * @param str String with or without single quotes at the beginning and end
     * @return String with single quotes at the beginning and end
     * 
     */
    public static String addSingleQuotes(String str) {
        return "'"+removeSingleQuotes(str)+"'";
    }
    
    
    /**
     * Splits an IRI to extract the identifier part after the 
     * hash sign '#'
     * 
     * @param iri IRI of the form http://...#value
     * @return The identifier value
     * @deprecated use {@link OWLThing} valueFromIRI instead
     */
    public static String valueFromIRI(String iri) {
        String[] ks = iri.split("#");
           if(ks.length>1) {
               String res = ks[1].replaceAll("'", "");
               return res;
           }
           else return iri;
     }
    
    /**
     * Splits an IRI to extract the base IRI part before the 
     * hash sign '#'
     * 
     * @param iri IRI of the form http://...#value
     * @return The IRI base
     * @deprecated use {@link OWLThing} prefixFromIRI instead
     */
	public static String prefixFromIRI(String iri) {
		
        String[] ks = iri.split("#");
        if(ks.length>1) {
            String res = ks[0].replaceAll("'", "");
            return res;
        }
        else return iri;
	}
	
    /**
     * Splits an IRI to extract the filename after 
     * the last slash
     * 
     * @param iri IRI of the form http://.../file.name
     * @return The filename file.name
     * @deprecated use {@link OWLThing} fileNameFromIRI instead
     */
	public static String fileNameFromIRI(String iri) {
		
        String[] ks = iri.split("/");
        if(ks.length>1) {
            String res = ks[ks.length-1].replaceAll("'", "");
            return res;
        }
        else return iri;
	}
	
    
    /**
     * Remove the literal(type('...', )) wrapper around literals coming from
     * Prolog
     * 
     * @param typedValue Value that potentially has a literal() spec around it
     * @return cleaned value
     */
    public static String stripLiteralType(String typedValue) {

    	if(!typedValue.startsWith("literal("))
    		return typedValue;
    	
    	typedValue = typedValue.substring(13, typedValue.length()-2);

    	// remove literal(type( and closing parentheses
    	
    	String[] parts = typedValue.split(",");
    	String[] clean = Arrays.copyOfRange(parts, 1, parts.length);
    	String cleanValue = Joiner.on(",").join(clean);
    	
    	if(cleanValue.startsWith(" "))
    		cleanValue = cleanValue.substring(1);
    	
    	return removeSingleQuotes(cleanValue);
    }

    
    /**
     * Retrieve the datatype / unit of measure from the literal(...) spec 
     * 
     * @param typedValue Value that potentially has a literal() spec around it
     * @return IRI of the datatype or null if it cannot be determined
     */
    public static String getLiteralType(String typedValue) {

    	if(!typedValue.startsWith("literal("))
    		return null;
    	
    	typedValue = typedValue.substring(13, typedValue.length()-2);

    	String[] parts = typedValue.split(",");
    	String cleanValue = parts[0];
    	
    	if(cleanValue.startsWith(" "))
    		cleanValue = cleanValue.substring(1);
    	
    	return removeSingleQuotes(cleanValue);
    }
    
	
    /**
     * Check if Prolog engine has been initialized.
     * 
     * @return true if initialized
     */
    public static boolean isInitialized() {
		return initialized;
	}


    /**
     * Set initialization status of the Prolog engine, for example if it has
     * been initialized using external methods.
     * 
     * @param initialized true if initialized
     */
	public static void setInitialized(boolean initialized) {
		PrologInterface.initialized = initialized;
	}
    
}

