package edu.tum.cs.ias.knowrob.vis.actions;

public enum FsmActionResult {
	
    /**
     * Execution successful.
     */
    OK,
    /**
     * Condition check returned 'true'.
     */
    CONDITION_TRUE,
    /**
     * Condition check returned 'false'.
     */
    CONDITION_FALSE,
    /**
     * Generic, unspecified error.
     */
    ERROR,
    /**
     * The operation was aborted.
     */
    ABORT,
    /**
     * Service ran out of the resources needed to complete the operation.
     */
    OUT_OF_RESOURCES,
    /**
     * The operation timed out.
     */
    TIMEOUT;
    
    
    public static FsmActionResult fromString(String status) {
    	
    	if(status.endsWith("OK")) {
    		return FsmActionResult.OK;
    		
    	} else if(status.endsWith("CONDITION_TRUE")) {
    		return FsmActionResult.CONDITION_TRUE;
    		
    	} else if(status.endsWith("CONDITION_FALSE")) {
    		return FsmActionResult.CONDITION_FALSE;
    		
    	} else if(status.endsWith("TIMEOUT")) {
    		return FsmActionResult.TIMEOUT;
    		
    	} else if(status.endsWith("OUT_OF_RESOURCES")) {
    		return FsmActionResult.OUT_OF_RESOURCES;
    		
    	} else if(status.endsWith("ABORT")) {
    		return FsmActionResult.ABORT;
    		
    	} else {
    		return FsmActionResult.ERROR;
    	}
	}
    
    public static String toOWLIdentifier(FsmActionResult r) {
    	
    	String res = r.toString();
    	
    	if(!res.startsWith("CONDITION"))
    		res = "COMPLETED_" + res;
    	
    	return "http://ias.cs.tum.edu/kb/knowrob.owl#" + res;
    }

}

