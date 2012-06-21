/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.util;

import java.awt.Color;
import java.io.Serializable;

import javax.vecmath.Point3f;

import processing.core.PGraphics;

/**
 * Base class for all drawable model parts (Line / Triangle)
 * 
 * @author Stefan Profanter
 * 
 */
public abstract class DrawObject implements Serializable {
	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= -1917773602783043823L;

	/**
	 * Multiplies the two given matrix. Must have correct size for multiplying.
	 * 
	 * @param mat1
	 *            matrix 1
	 * @param mat2
	 *            matrix 2
	 * @return the result of multiplication
	 */
	public static float[][] MatrixMultiply(float mat1[][], float mat2[][]) {
		int x = mat1.length;
		int y = mat2.length;
		float result[][] = new float[x][y];

		for (int i = 0; i < x; i++) {
			for (int j = 0; j < y - 1; j++) {
				for (int k = 0; k < y; k++) {

					result[i][j] += mat1[i][k] * mat2[k][j];
				}
			}
		}

		return result;
	}

	/**
	 * the position points of the object
	 */
	protected Point3f		position[];

	/**
	 * Color or texture of the object
	 */
	protected Appearance	appearance;

	/**
	 * Constructor which initializes position array to <code>numberOfEdges</code> items.
	 * 
	 * @param numberOfEdges
	 *            number of edges. Line: 2, Triangle: 3
	 */
	public DrawObject(final int numberOfEdges) {
		position = new Point3f[numberOfEdges];
	}

	/**
	 * Apply the color of appearance member to the PApplet. Called before drawing a DrawObject.
	 * 
	 * @param g
	 *            Graphics to draw on
	 * @param overrideColor
	 *            If != null this color is taken instead of the color from appearance
	 */
	protected void applyColor(PGraphics g, Color overrideColor) {
		if (appearance.getColorLine() != null) {
			if (overrideColor != null)
				g.stroke(overrideColor.getRed(), overrideColor.getGreen(), overrideColor.getBlue(),
						overrideColor.getAlpha());
			else
				g.stroke(appearance.getColorLine().getRed(), appearance.getColorLine().getGreen(),
						appearance.getColorLine().getBlue(), appearance.getColorLine().getAlpha());
			g.strokeWeight(appearance.getStrokeWeight());
		} else {
			g.noStroke();
		}

		if (overrideColor != null)
			g.fill(overrideColor.getRed(), overrideColor.getGreen(), overrideColor.getBlue(),
					overrideColor.getAlpha());
		else if (appearance.getImageReference() == null) {
			if (appearance.getColorFill() != null) {
				g.fill(appearance.getColorFill().getRed(), appearance.getColorFill().getGreen(),
						appearance.getColorFill().getBlue(), appearance.getColorFill().getAlpha());
			} else {
				g.noFill();
			}
		} else {
			// Has texture
			// Use fallback if texture isn't drawn. So fill triangles with white color
			g.fill(255, 255, 255, 0);
		}
	}

	/**
	 * @return the appearance
	 */
	public Appearance getAppearance() {
		return appearance;
	}

	/**
	 * @return the position
	 */
	public Point3f[] getPosition() {
		return position;
	}

	/**
	 * Scales all coordinates of the position points by the given factor
	 * 
	 * @param factor
	 *            The Scale factor
	 */
	public void scale(float factor) {
		for (int v = 0; v < position.length; v++) {
			position[v].x *= factor;
			position[v].y *= factor;
			position[v].z *= factor;
		}
		updateNormalVector();
	}

	/**
	 * @param appearance
	 *            the appearance to set
	 */
	public void setAppearance(Appearance appearance) {
		this.appearance = appearance;
	}

	/**
	 * Sets the position array of this object and calls <code>updateNormalVector</code>
	 * 
	 * @param position
	 *            new position of this object.
	 */
	public void setPosition(Point3f[] position) {
		this.position = position;
		updateNormalVector();
	}

	/**
	 * Apply 4x4 transformation matrix to the position vectors of this object
	 * 
	 * @param matrix
	 *            the transformation matrix
	 */
	public void transform(float[][] matrix) {
		for (int v = 0; v < position.length; v++) {
			float[] newPos = new float[4];
			for (int row = 0; row < 4; row++) {
				newPos[row] = position[v].x * matrix[row][0] + position[v].y * matrix[row][1]
						+ position[v].z * matrix[row][2] + matrix[row][3];
			}
			position[v].x = newPos[0] / newPos[3];
			position[v].y = newPos[1] / newPos[3];
			position[v].z = newPos[2] / newPos[3];
		}
		updateNormalVector();
	}

	/**
	 * Moves the triangle in the given direction.
	 * 
	 * @param x
	 *            direction in x
	 * @param y
	 *            direction in y
	 * @param z
	 *            direction in z
	 */
	public void translate(float x, float y, float z) {
		for (int v = 0; v < position.length; v++) {
			position[v].x += x;
			position[v].y += y;
			position[v].z += z;
		}
	}

	/**
	 * Recalculates the normal vector and centroid. Called automatically when calling
	 * <code>setPosition</code>. If you modify the position array directly, call this afterwards.
	 * 
	 * On Line this function has no effect.
	 */
	public void updateNormalVector() {
		/*
		 * Overridden in triangles class, line doesn't have a normal vector
		 */
	}

}
