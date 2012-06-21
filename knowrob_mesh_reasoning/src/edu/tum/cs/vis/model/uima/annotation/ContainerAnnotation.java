/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation;

import java.awt.Color;

import javax.vecmath.Vector3f;

/**
 * @author Stefan Profanter
 * 
 */
public class ContainerAnnotation extends MeshAnnotation {

	/**
	 * 
	 */
	private static final long	serialVersionUID	= 7483463728879308919L;

	private Vector3f			direction;

	private float				volume;

	/**
	 * @param annotationColor
	 */
	public ContainerAnnotation() {
		super(new Color(27, 93, 27));
		// TODO Auto-generated constructor stub
	}

	/**
	 * @return the direction
	 */
	public Vector3f getDirection() {
		return direction;
	}

	/**
	 * @return the volume
	 */
	public float getVolume() {
		return volume;
	}

	/**
	 * @param direction
	 *            the direction to set
	 */
	public void setDirection(Vector3f direction) {
		this.direction = direction;
	}

	/**
	 * @param volume
	 *            the volume to set
	 */
	public void setVolume(float volume) {
		this.volume = volume;
	}

}
