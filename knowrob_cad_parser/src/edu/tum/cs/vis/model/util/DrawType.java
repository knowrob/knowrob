/*******************************************************************************
 * Copyright (c) 2013 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2013
 ******************************************************************************/
package edu.tum.cs.vis.model.util;

/**
 * Draw type for DrawSettings.
 * 
 * 
 * @author Stefan Profanter
 * 
 * @see edu.tum.cs.vis.model.util.DrawSettings
 * 
 */
public enum DrawType {
	/**
	 * Fill triangles
	 */
	FILL, /**
	 * Draw contour lines only
	 */
	LINES,
	/**
	 * Draw vertex points only
	 */
	POINTS
}
