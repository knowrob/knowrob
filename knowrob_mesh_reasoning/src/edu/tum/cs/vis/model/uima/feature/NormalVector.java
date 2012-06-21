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
package edu.tum.cs.vis.model.uima.feature;

import javax.vecmath.Vector3d;

/**
 * Represents a normal vector
 * 
 * @author Stefan Profanter
 * 
 */
public class NormalVector extends Vector3d {

	/**
	 * Auto generated
	 */
	private static final long	serialVersionUID	= -3151983130243696661L;

	/**
	 * Default constructor
	 */
	public NormalVector() {
		super();
	}

	/**
	 * Constructor with direct initialization
	 * 
	 * @param normalVector
	 *            value to initialize
	 */
	public NormalVector(Vector3d normalVector) {
		super(normalVector);
	}

	@Override
	public String toString() {
		return String.format("(%.3f,%.3f,%.3f)", x, y, z);
	}

}
