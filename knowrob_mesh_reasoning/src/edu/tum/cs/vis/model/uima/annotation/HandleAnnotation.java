/*******************************************************************************
 * Copyright (c) 2013 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2013
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation;

import edu.tum.cs.vis.model.uima.annotation.primitive.Cone;

/**
 * Basic handle annotation interface. Cone and complex handle annotation implement this interface.
 * 
 * @author Stefan Profanter
 * 
 */
public interface HandleAnnotation {
	/**
	 * Get area coverage of handle. Calculated by dividing sum of triangle area by the area of the
	 * cone.
	 * 
	 * @return area coverage. Value of 1 means cone and triangles have the same area.
	 */
	public float getAreaCoverage();

	/**
	 * Get the cone which represents the handle.
	 * 
	 * @return the cone
	 */
	public Cone getCone();
}
