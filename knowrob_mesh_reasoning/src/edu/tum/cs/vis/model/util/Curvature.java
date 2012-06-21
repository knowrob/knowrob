/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.util;

import java.awt.Color;

import javax.vecmath.Vector3f;

import edu.tum.cs.vis.model.uima.annotation.primitive.PrimitiveType;

/**
 * @author Stefan Profanter
 * 
 */
public class Curvature {

	/**
	 * Convert hsv to rgb color space acording to:
	 * https://en.wikipedia.org/wiki/HSL_and_HSV#From_HSV
	 * 
	 * @param h
	 * @param s
	 * @param v
	 * @return
	 */
	private static Color hsv2srgb(float h, float s, float v) {
		// From FvD
		float H = h, S = s, V = v;
		if (S <= 0.0f)
			return new Color(V, V, V);
		H = (float) (H % (2 * Math.PI));
		if (H < 0.0f)
			H += 2 * Math.PI;
		// S and V is now between 0 and 1, H between 0 an 2*PI

		float hi = (float) (H * (Math.PI / 3)); // Divide by 60 degree
		int i = (int) Math.floor(hi);
		float f = hi - i;
		float p = V * (1.0f - S);
		float q = V * (1.0f - (S * f));
		float t = V * (1.0f - (S * (1.0f - f)));
		switch (i) {
			case 0:
				return new Color(V, t, p);
			case 1:
				return new Color(q, V, p);
			case 2:
				return new Color(p, V, t);
			case 3:
				return new Color(p, q, V);
			case 4:
				return new Color(t, p, V);
			default:
				return new Color(V, p, q);
		}
	}

	private Vector3f		principleDirectionMin	= new Vector3f();

	private Vector3f		principleDirectionMax	= new Vector3f();
	private float			curvatureMin			= 0;

	private float			curvatureMax			= 0;

	private float			curvatureMinMax			= 0;

	private PrimitiveType	primitiveType;
	private float			hue;

	private float			saturation;

	public Color getColor() {
		return hsv2srgb(hue, saturation, 1);
	}

	/**
	 * @return the curvatureMax
	 */
	public float getCurvatureMax() {
		return curvatureMax;
	}

	/**
	 * @return the curvatureMin
	 */
	public float getCurvatureMin() {
		return curvatureMin;
	}

	/**
	 * @return the curvatureMinMax
	 */
	public float getCurvatureMinMax() {
		return curvatureMinMax;
	}

	/**
	 * @return the hue
	 */
	public float getHue() {
		return hue;
	}

	/**
	 * @return the primitiveType
	 */
	public PrimitiveType getPrimitiveType() {
		return primitiveType;
	}

	/**
	 * @return the principleDirectionMax
	 */
	public Vector3f getPrincipleDirectionMax() {
		return principleDirectionMax;
	}

	/**
	 * @return the principleDirectionMin
	 */
	public Vector3f getPrincipleDirectionMin() {
		return principleDirectionMin;
	}

	/**
	 * @return the saturation
	 */
	public float getSaturation() {
		return saturation;
	}

	/**
	 * @param curvatureMax
	 *            the curvatureMax to set
	 */
	public void setCurvatureMax(float curvatureMax) {
		this.curvatureMax = curvatureMax;
	}

	/**
	 * @param curvatureMin
	 *            the curvatureMin to set
	 */
	public void setCurvatureMin(float curvatureMin) {
		this.curvatureMin = curvatureMin;
	}

	/**
	 * @param curvatureMinMax
	 *            the curvatureMinMax to set
	 */
	public void setCurvatureMinMax(float curvatureMinMax) {
		this.curvatureMinMax = curvatureMinMax;
	}

	/**
	 * @param hue
	 *            the hue to set
	 */
	public void setHue(float hue) {
		this.hue = hue;
	}

	/**
	 * @param primitiveType
	 *            the primitiveType to set
	 */
	public void setPrimitiveType(PrimitiveType primitiveType) {
		this.primitiveType = primitiveType;
	}

	/**
	 * @param principleDirectionMax
	 *            the principleDirectionMax to set
	 */
	public void setPrincipleDirectionMax(Vector3f principleDirectionMax) {
		this.principleDirectionMax = principleDirectionMax;
	}

	/**
	 * @param principleDirectionMin
	 *            the principleDirectionMin to set
	 */
	public void setPrincipleDirectionMin(Vector3f principleDirectionMin) {
		this.principleDirectionMin = principleDirectionMin;
	}

	/**
	 * @param saturation
	 *            the saturation to set
	 */
	public void setSaturation(float saturation) {
		this.saturation = saturation;
	}

}
