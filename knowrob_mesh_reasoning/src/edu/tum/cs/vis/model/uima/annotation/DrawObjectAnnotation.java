/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation;

import java.awt.Color;

import edu.tum.cs.vis.model.util.DrawObject;

/**
 * Annotation on a single DrawObject (Triangle, Line).
 * 
 * @author Stefan Profanter
 * 
 * @see DrawObject
 * 
 */
public abstract class DrawObjectAnnotation extends DrawableAnnotation {

	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= -1302487276536212663L;

	/**
	 * Object for which this annotation is
	 */
	protected DrawObject		object;

	public DrawObjectAnnotation(Color annotationColor) {
		super(annotationColor);
	}

	/**
	 * Get object for which this annotation is.
	 * 
	 * @return the object
	 */
	public DrawObject getObject() {
		return object;
	}

	/**
	 * Set object for which this annotation is.
	 * 
	 * @param object
	 *            the object to set
	 */
	public void setObject(DrawObject object) {
		this.object = object;
	}

}
