package edu.tum.cs.vis.model.parser;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Iterator;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

import edu.tum.cs.util.ResourceRetriever;
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
	 * Checks if the given filename is an uri or a local filename.
	 * If local: the filename is simply returned
	 * If uri: The file will be retrieved to tmp directory by calling retrieveFile(String,String)
	 * 			and the tmp path will be returned
	 * @param filename Local file or Uri to retrieve. For valid format see http://www.ros.org/wiki/resource_retriever
	 * @return filename if is local, path to retrieved file if is url
	 */
	public static String retrieveFile(String filename)
	{
		if (filename.indexOf("://")>0)
		{
			int idx = Math.max(filename.lastIndexOf('/'), filename.lastIndexOf('\\'));
			String file = "";
			if (idx <= 0)
				file = "retrievedFile";
			else
				file = filename.substring(idx+1);
			File tmpPath = ResourceRetriever.retrieve(filename);
			if (tmpPath != null)
				return tmpPath.getAbsolutePath();
			else
			{
				System.out.println("Couldn't retrieve file: " + filename);
				return null;
			}
		}
		return filename;
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

	/**
	 * Unzip a zipped file (eg. kmz) into given directory 
	 * @param zipFile zipped file to unzip
	 * @param outputDirectory destination directory for unzipped content
	 * @return true if successfully unzipped
	 */
	public static boolean Unzip(String zipFile, String outputDirectory) {
		if (!outputDirectory.endsWith("/") && !outputDirectory.endsWith("\\"))
			outputDirectory += "/";

		BufferedOutputStream dest = null;
		BufferedInputStream is = null;
		int BUFFER = 2048;
		ZipEntry entry;
		ZipFile zipfile;
		try {
			zipfile = new ZipFile(zipFile);
			Enumeration<? extends ZipEntry> e = zipfile.entries();
			while (e.hasMoreElements()) {
				entry = (ZipEntry) e.nextElement();
				if (entry.isDirectory()) {
					(new File(outputDirectory + entry.getName())).mkdir();
					continue;
				}

				String filename = outputDirectory + entry.getName();
				String filePath = filename.substring(0,
						filename.lastIndexOf(File.separator));

				// Create directory if not existing
				if (!(new File(filePath)).exists()) {
					(new File(filePath)).mkdirs();
				}
				is = new BufferedInputStream(zipfile.getInputStream(entry));
				int count;
				byte data[] = new byte[BUFFER];
				FileOutputStream fos = new FileOutputStream(filename);
				dest = new BufferedOutputStream(fos, BUFFER);
				while ((count = is.read(data, 0, BUFFER)) != -1) {
					dest.write(data, 0, count);
				}
				dest.flush();
				dest.close();
				is.close();
			}
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			System.err.println("Couldn't unzip file: " + zipFile);
			e1.printStackTrace();
			return false;
		}
		return true;
	}

}
