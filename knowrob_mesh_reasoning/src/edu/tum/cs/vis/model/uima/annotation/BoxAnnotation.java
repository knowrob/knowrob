/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation;

import java.awt.Color;

import javax.vecmath.Point3f;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

import edu.tum.cs.vis.model.Model;

/**
 * 
 * Mesh Annotation for a 6-sided rectangular box. Box is annotated by position and 3 vectors
 * indicating width, depth and height.
 * 
 * @author Stefan Profanter
 * 
 */
public class BoxAnnotation extends MeshAnnotation<BoxAnnotation> {

	/**
	 * generated version uid
	 */
	private static final long	serialVersionUID	= -2348172880738880568L;

	/**
	 * This is the center of the box
	 */
	private Point3f				center;

	/**
	 * Vector from center to left side plane (= half width)
	 */
	private Vector3f			directionX;

	/**
	 * Vector from center to back plane (= half depth)
	 */
	private Vector3f			directionY;

	/**
	 * Vector from center to top plane (= half height)
	 */
	private Vector3f			directionZ;

	/**
	 * @param model
	 *            parent model for annotation
	 */
	public BoxAnnotation(Model model) {
		super(BoxAnnotation.class, model, new Color(0, 0, 255));
	}

	/**
	 * @return the center
	 */
	public Point3f getCenter() {
		return center;
	}

	/**
	 * @return the center unscaled
	 */
	public Tuple3f getCenterUnscaled() {
		return model.getUnscaled(center);
	}

	/**
	 * @return the directionX
	 */
	public Vector3f getDirectionX() {
		return directionX;
	}

	/**
	 * @return the directionX unscaled
	 */
	public Vector3f getDirectionXUnscaled() {
		return model.getUnscaled(directionX);
	}

	/**
	 * @return the directionY
	 */
	public Vector3f getDirectionY() {
		return directionY;
	}

	/**
	 * @return the directionY unscaled
	 */
	public Vector3f getDirectionYUnscaled() {
		return model.getUnscaled(directionY);
	}

	/**
	 * @return the directionZ
	 */
	public Vector3f getDirectionZ() {
		return directionZ;
	}

	/**
	 * @return the directionZ unscaled
	 */
	public Vector3f getDirectionZUnscaled() {
		return model.getUnscaled(directionZ);
	}

	/**
	 * @param center
	 *            the center to set
	 */
	public void setCenter(Point3f center) {
		this.center = center;
	}

	/**
	 * @param directionX
	 *            the directionX to set
	 */
	public void setDirectionX(Vector3f directionX) {
		this.directionX = directionX;
	}

	/**
	 * @param directionY
	 *            the directionY to set
	 */
	public void setDirectionY(Vector3f directionY) {
		this.directionY = directionY;
	}

	/**
	 * @param directionZ
	 *            the directionZ to set
	 */
	public void setDirectionZ(Vector3f directionZ) {
		this.directionZ = directionZ;
	}

}
