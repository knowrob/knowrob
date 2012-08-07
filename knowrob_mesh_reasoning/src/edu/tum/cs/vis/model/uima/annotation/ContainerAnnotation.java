/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation;

import java.awt.Color;

import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

import edu.tum.cs.vis.model.Model;

/**
 * Mesh annotation for container. A container is like a vessel where a bottom cap and walls are
 * found.
 * 
 * 
 * @author Stefan Profanter
 * 
 */
public class ContainerAnnotation extends MeshAnnotation<ContainerAnnotation> {

	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= 7483463728879308919L;

	/**
	 * Direction of container. Shows along generating line into direction, where the container is
	 * open. Length of this vector is exactly containers height.
	 */
	private Vector3f			direction;

	/**
	 * total volume of container
	 */
	private float				volume;

	/**
	 * Constructor for a container
	 * 
	 * @param model
	 *            parent model for annotation
	 */
	public ContainerAnnotation(Model model) {
		super(ContainerAnnotation.class, model, new Color(27, 93, 27));
	}

	/**
	 * Direction of container. Shows along generating line into direction, where the container is
	 * open. Length of this vector is exactly containers height.
	 * 
	 * @return the direction
	 */
	public Vector3f getDirection() {
		return direction;
	}

	/**
	 * Direction of container. Shows along generating line into direction, where the container is
	 * open. Length of this vector is exactly containers height as unscaled value.
	 * 
	 * @return the direction unscaled
	 */
	public Tuple3f getDirectionUnscaled() {
		return model.getUnscaled(direction);
	}

	/**
	 * Volume of container
	 * 
	 * @return the volume
	 */
	public float getVolume() {
		return volume;
	}

	/**
	 * Volume of container as unscaled value.
	 * 
	 * @return the volume
	 */
	public float getVolumeUnscaled() {
		return model.getUnscaled(volume);
	}

	/**
	 * Set direction of container. Length should be height of the container and vector should point
	 * into opening direction.
	 * 
	 * @param direction
	 *            the direction to set
	 */
	public void setDirection(Vector3f direction) {
		this.direction = direction;
	}

	/**
	 * Set volume of container
	 * 
	 * @param volume
	 *            the volume to set
	 */
	public void setVolume(float volume) {
		this.volume = volume;
	}

}
