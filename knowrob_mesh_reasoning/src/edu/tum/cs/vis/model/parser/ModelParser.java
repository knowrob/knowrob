/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.parser;

import java.util.HashMap;
import java.util.Iterator;

import edu.tum.cs.vis.model.Model;

/**
 * Base class for all ModelParsers. Used to parse models from file an draw them onto the Processing
 * applet.
 * 
 * @author Stefan Profanter
 * 
 */
public abstract class ModelParser {

	/**
	 * Contains for each valid file extension the appropriate parser. Used to automatically
	 * determine the correct parser for a file.
	 */
	private static final HashMap<String, Class<? extends ModelParser>>	extensionAssignment	= new HashMap<String, Class<? extends ModelParser>>();
	static {
		/*
		 * Add here other parsers with file extension
		 */
		extensionAssignment.put("dae", ColladaParser.class);
		extensionAssignment.put("kmz", ColladaParser.class);
		extensionAssignment.put("mesh", CustomParser.class);
	}

	/**
	 * This is a buffer for already parsed models. If a model is loaded through parseModel method it
	 * will be stored in this HashMap. It it is loaded a second time, instead of parsing it again it
	 * will be taken from this HashMap
	 */
	private static HashMap<String, Model>								modelBuffer			= new HashMap<String, Model>();

	/**
	 * Finds the appropriate parser for the given filename by comaring the file extension.
	 * 
	 * @param filename
	 *            Filename to find parser for
	 * @return the Parser Class for this file
	 */
	public static Class<? extends ModelParser> findParser(String filename) {
		return extensionAssignment.get(getExtension(filename));
	}

	/**
	 * Gets the filename extension of a filename. Usually the last 3 characters
	 * 
	 * @param filename
	 *            The filename
	 * @return the extension without dot
	 */
	protected static String getExtension(String filename) {
		return filename.substring(filename.lastIndexOf('.') + 1);
	}

	/**
	 * Contains mesh of the model
	 */
	protected Model	model;

	/**
	 * Checks if the file extension is valid for this class
	 * 
	 * @param filename
	 *            file to check
	 * @return true if extension is valid for this class. false otherwise
	 */
	protected boolean checkExtension(String filename) {
		if (!isValidExtension(filename)) {
			System.out.println("Unknown file extension for class: " + this.getClass().getName()
					+ " " + filename + "\n Must be one of: " + getValidExtensions());
			return false;
		}
		return true;
	}

	/**
	 * Get the object containing the model (meshes)
	 * 
	 * @return the model member
	 */
	public Model getModel() {
		return model;
	}

	/**
	 * Returns a comma separated string with all valid file extensions of this class
	 * 
	 * @return List of file extensions: eg "kmz, dae,"
	 */
	protected String getValidExtensions() {
		Iterator<String> it = extensionAssignment.keySet().iterator();
		String extensions = "";
		while (it.hasNext()) {
			String key = it.next();
			Object val = extensionAssignment.get(key);
			if (val.getClass() == this.getClass())
				extensions += key + ", ";
		}
		return extensions;
	}

	/**
	 * Checks if the given filename has a valid extension for the extended class.
	 * 
	 * @param filename
	 *            Filename to check
	 * @return true if extension is valid
	 */
	protected boolean isValidExtension(String filename) {
		if (extensionAssignment.get(getExtension(filename)) == this.getClass())
			return true;
		return false;
	}

	/**
	 * This function will be called from parseModel if the model isn't in the Buffer
	 * 
	 * @param filename
	 *            Physical file of model to parse
	 * @return true if model parsed successfully. false otherwise
	 */
	protected abstract boolean loadModel(String filename);

	/**
	 * Main function to parse the model from the given filename. If model has already been parsed,
	 * it will be taken from modelBuffer instead of parsing it again.
	 * 
	 * @param filename
	 *            Physical file of model to parse
	 * @return true if model parsed successfully. false otherwise
	 */
	public boolean parseModel(String filename) {
		Model m = modelBuffer.get(filename);
		boolean retVal = true;

		if (m == null) {
			model = new Model();
			retVal = loadModel(filename);
			modelBuffer.put(filename, model);
			model.getGroup().initialize(model.getTextureBasePath());
		} else
			model = m;
		return retVal;
	}

}
