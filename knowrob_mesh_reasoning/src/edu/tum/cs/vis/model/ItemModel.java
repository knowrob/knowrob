/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors:
 *     Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model;

import edu.tum.cs.vis.model.parser.ModelParser;

/**
 * Holds information about the 3D Model of an ItemBase object.
 * 
 * @author Stefan Profanter
 * 
 */
public class ItemModel {

	/**
	 * Physical file path to the model
	 */
	private String		path;

	/**
	 * The parser for the model.
	 */
	private ModelParser	parser;

	/**
	 * Constructor. The model will immediately be parsed and hold in RAM.
	 * 
	 * @param path
	 *            Path to the model
	 */
	public ItemModel(String path) {
		this.path = path;
		if (!parseModel())
			parser = null;
	}

	/**
	 * Get associated model parser which contains the model for this item
	 * 
	 * @return the model parser
	 */
	public ModelParser getParser() {
		return parser;
	}

	/**
	 * Get physical file path of model
	 * 
	 * @return physical file path
	 */
	public String getPath() {
		return path;
	}

	/**
	 * Selects automagically the correct parser for the file (by file extension) and parses the
	 * model with the selected parser.
	 * 
	 * @return true if successfully parsed, false otherwise
	 */
	public boolean parseModel() {
		if (path == null)
			return false;
		Class<? extends ModelParser> cl = ModelParser.findParser(path);
		if (cl == null) {
			System.out.println("No parser found for: " + path);
			return false;
		}
		try {
			parser = cl.newInstance();
		} catch (InstantiationException e) {
			System.out.println("Couldn't instantiate parser for: " + path);
			e.printStackTrace();
		} catch (IllegalAccessException e) {
			System.out.println("Couldn't instantiate parser for: " + path);
			e.printStackTrace();
		}
		return parser.parseModel(path);
	}

	/**
	 * Set associated model parser which contains the model for this item
	 * 
	 * @param parser
	 *            the model parser
	 */
	public void setParser(ModelParser parser) {
		this.parser = parser;
	}

	/**
	 * Set physical file path of model
	 * 
	 * @param path
	 *            physical file path
	 */
	public void setPath(String path) {
		this.path = path;
	}

}
