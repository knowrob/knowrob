/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2013
 ******************************************************************************/
package edu.tum.cs.tools.ImageGenerator;

/**
 * Interface for a specific image generator action.
 * 
 * @author Stefan Profanter
 * 
 */
public interface ImageGeneratorAction {
	/**
	 * The action method.
	 * 
	 * @param settings
	 *            ImageGeneratorSettings of current MeshReasoning process.
	 */
	public void trigger(ImageGeneratorSettings settings);
}
