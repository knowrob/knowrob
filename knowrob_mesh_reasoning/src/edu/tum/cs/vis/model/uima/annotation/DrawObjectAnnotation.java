/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation;

import processing.core.PGraphics;
import edu.tum.cs.vis.model.util.DrawObject;

/**
 * @author Stefan Profanter
 * 
 */
public class DrawObjectAnnotation extends DrawableAnnotation {

	/**
	 * 
	 */
	private static final long	serialVersionUID	= -1302487276536212663L;
	protected DrawObject		object;

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.DrawableAnnotation#drawAnnotation(processing.core.PGraphics)
	 */
	@Override
	protected void drawAnnotation(PGraphics g) {
		// TODO Auto-generated method stub

	}

	/**
	 * @return the object
	 */
	public DrawObject getObject() {
		return object;
	}

	/**
	 * @param object
	 *            the object to set
	 */
	public void setObject(DrawObject object) {
		this.object = object;
	}

}
