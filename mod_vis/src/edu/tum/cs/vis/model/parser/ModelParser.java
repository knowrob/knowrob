package edu.tum.cs.vis.model.parser;

import java.util.HashMap;
import java.util.Iterator;

import edu.tum.cs.vis.model.util.Group;

import processing.core.PApplet;

/**
 * Base class for all ModelParsers. Used to parse models from file an draw them onto the Processing applet.
 * 
 * @author Stefan Profanter
 *
 */
public abstract class ModelParser {
	
	/**
	 * Contains for each valid file extension the appropriate parser.
	 * Used to automatically determine the correct parser for a file.
	 */
	private static final HashMap<String,Class<? extends ModelParser>> extensionAssignment = new HashMap<String,Class<? extends ModelParser>>();
    static {
    	/*
    	 * Add here other parsers with file extension
    	 */
    	extensionAssignment.put("dae", ColladaParser.class);
    	extensionAssignment.put("kmz", ColladaParser.class);
    }
    
    /**
     * This is a buffer for already parsed models.
     * If a model is loaded through parseModel method it will be stored in this HashMap.
     * It it is loaded a second time, instead of parsing it again it will be taken from this HashMap
     */
    private static HashMap<String,Group> modelBuffer = new HashMap<String,Group>();
       
    /**
     * Contains mesh of the model
     */
    protected Group group;

    public Group getGroup() {
		return group;
	}

	/**
     * Gets the filename extension of a filename. Usually the last 3 characters
     * @param filename The filename
     * @return the extension without dot
     */
    protected static String getExtension(String filename)
    {
    	return filename.substring(filename.lastIndexOf('.') + 1);
    }
    
    /**
     * Checks if the given filename has a valid extension for the extended class.
     * @param filename Filename to check
     * @return true if extension is valid
     */
    protected boolean isValidExtension(String filename)
    {
    	if (extensionAssignment.get(getExtension(filename)) == this.getClass())
    		return true;
    	return false;
    }
    
    /**
     * Checks if the file extension is valid for this class
     * @param filename file to check
     */
    protected boolean checkExtension(String filename) {
    	if (!isValidExtension(filename))
    	{
    		System.out.println("Unknown file extension for class: " + this.getClass().getName() + " " + filename + "\n Must be one of: " + getValidExtensions());
    		return false;
    	} else {
    		return true;
    	}
    }
    
    
    /**
     * Returns a komma separated string with all valid file extensions of this class
     * @return List of file extensions: eg "kmz, dae,"
     */
    protected String getValidExtensions() {
    	Iterator<String> it = extensionAssignment.keySet().iterator();
    	String extensions = "";
    	while(it.hasNext()) {
    		String key = it.next();
    		Object val = extensionAssignment.get(key);
    		if (val.getClass() == this.getClass())
    			extensions += key + ", "; 
    	}
    	return extensions;
    }

	/**
	 * Draw method to draw the model on the applet.
	 * @param applet The applet to draw on.
	 * @param colorOverride override the draw color an texture. Draw whole object in the given color if != 0
	 */
	public abstract void draw(PApplet applet, int colorOverride);

	/**
	 * This function will be called from parseModel if the model isn't in the Buffer
	 * @param filename Physical file of model to parse
	 */
	protected abstract boolean loadModel(String filename);
	
	/**
	 * Main function to parse the model from the given filename.
	 * If model has already been parsed, it will be taken from modelBuffer instead of parsing it again.
	 * @param filename Physical file of model to parse
	 * @return
	 */
	public boolean parseModel(String filename)
	{
		Group g = modelBuffer.get(filename);
		boolean retVal = true;
		
		if (g == null)
		{
			retVal =  loadModel(filename);
			modelBuffer.put(filename, this.group);
		}
		else
			this.group = g;
		return retVal;
	}
	
	/**
	 * Finds the appropriate parser for the given filename by comaring the file extension.
	 * @param filename Filename to find parser for
	 * @return the Parser Class for this file
	 */
	public static Class<? extends ModelParser> findParser(String filename)
	{
		return extensionAssignment.get(getExtension(filename));
	}

}
