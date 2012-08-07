/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation.primitive;

/**
 * Enum representing primitive types for vertices and triangles according to its curvature
 * properties.
 * 
 * @author Stefan Profanter
 * 
 */
public enum PrimitiveType {
	/**
	 * Flat face
	 */
	PLANE,
	/**
	 * convex sphere
	 */
	SPHERE_CONVEX,
	/**
	 * concave sphere
	 */
	SPHERE_CONCAVE,
	/**
	 * convex cone
	 */
	CONE_CONVEX,
	/**
	 * concave cone
	 */
	CONE_CONCAVE

}
