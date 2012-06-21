/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation.primitive;

import java.awt.Color;

import edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation;

/**
 * @author Stefan Profanter
 * 
 */
public class ConeAnnotation extends PrimitiveAnnotation {

	/**
	 * 
	 */
	private static final long	serialVersionUID	= -7420446109108464883L;

	private final boolean		concav;

	public ConeAnnotation(boolean concav) {
		super(concav ? new Color(0, 125, 125) : new Color(255, 255, 0));
		this.concav = concav;
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation#fit()
	 */
	@Override
	public void fit() {
		// TODO Auto-generated method stub

	}

}
