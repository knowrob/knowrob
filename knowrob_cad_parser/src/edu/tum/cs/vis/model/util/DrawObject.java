/*******************************************************************************
 * Copyright (c) 2013 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2013
 ******************************************************************************/
package edu.tum.cs.vis.model.util;

import java.io.Serializable;

import javax.vecmath.Vector3f;

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
	protected Vertex		position[];

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
		position = new Vertex[numberOfEdges];
	}

	/**
	 * Apply the color of appearance member to the PApplet. Called before drawing a DrawObject.
	 * 
	 * @param g
	 *            Graphics to draw on
	 * @param drawSettings
	 *            If != null this draw settings are used instead of the color from appearance
	 */
	protected void applyColor(PGraphics g, DrawSettings drawSettings) {
		if (appearance == null) {

			g.noStroke();
			g.noFill();
			if (drawSettings == null || drawSettings.drawType == DrawType.FILL) {
				if (drawSettings != null && drawSettings.getOverrideColor() != null)
					g.fill(drawSettings.getOverrideColor().getRed(), drawSettings
							.getOverrideColor().getGreen(), drawSettings.getOverrideColor()
							.getBlue(), drawSettings.getOverrideColor().getAlpha());
				else
					g.fill(200, 200, 200);
			} else {
				if (drawSettings.getOverrideColor() != null)
					g.stroke(drawSettings.getOverrideColor().getRed(), drawSettings
							.getOverrideColor().getGreen(), drawSettings.getOverrideColor()
							.getBlue(), drawSettings.getOverrideColor().getAlpha());
				else
					g.stroke(200, 200, 200);
				g.strokeWeight(drawSettings.getLineWidth());
			}
			return;
		}
		if (appearance.getColorLine() != null) {
			if (drawSettings != null && drawSettings.getOverrideColor() != null)
				g.stroke(drawSettings.getOverrideColor().getRed(), drawSettings.getOverrideColor()
						.getGreen(), drawSettings.getOverrideColor().getBlue(), drawSettings
						.getOverrideColor().getAlpha());
			else
				g.stroke(appearance.getColorLine().getRed(), appearance.getColorLine().getGreen(),
						appearance.getColorLine().getBlue(), appearance.getColorLine().getAlpha());
			g.strokeWeight(appearance.getStrokeWeight());
		} else {
			g.noStroke();
		}

		if (drawSettings == null || drawSettings.drawType == DrawType.FILL) {
			if (drawSettings != null && drawSettings.getOverrideColor() != null)
				g.fill(drawSettings.getOverrideColor().getRed(), drawSettings.getOverrideColor()
						.getGreen(), drawSettings.getOverrideColor().getBlue(), drawSettings
						.getOverrideColor().getAlpha());
			else if (appearance.getImageReference() == null) {
				if (appearance.getColorFill() != null) {
					g.fill(appearance.getColorFill().getRed(),
							appearance.getColorFill().getGreen(), appearance.getColorFill()
									.getBlue(), appearance.getColorFill().getAlpha() == 0 ? 255
									: appearance.getColorFill().getAlpha());
				} else {
					g.noFill();
				}
			} else {
				// Has texture
				// Use fallback if texture isn't drawn. So fill triangles with white color
				g.fill(255, 255, 255, 0);
			}
		} else {
			if (drawSettings != null && drawSettings.getOverrideColor() != null)
				g.stroke(drawSettings.getOverrideColor().getRed(), drawSettings.getOverrideColor()
						.getGreen(), drawSettings.getOverrideColor().getBlue(), drawSettings
						.getOverrideColor().getAlpha());
			else if (appearance.getColorFill() != null) {
				g.stroke(appearance.getColorFill().getRed(), appearance.getColorFill().getGreen(),
						appearance.getColorFill().getBlue(),
						appearance.getColorFill().getAlpha() == 0 ? 255 : appearance.getColorFill()
								.getAlpha());
			} else
				g.stroke(255, 255, 255);
			g.strokeWeight(drawSettings.getLineWidth());
		}
	}

	@Override
	public boolean equals(Object o) {
		if (o == this) {
			return true;
		}
		if (!(o instanceof DrawObject)) {
			return false;
		}

		DrawObject t = (DrawObject) o;
		if (t.position.length != t.position.length)
			return false;
		int cnt = 0;
		int maxCnt = position.length;
		for (int k = 0; k < maxCnt; k++) {
			for (int l = 0; l < maxCnt; l++) {
				if (t.position[k].equals(position[l]))
					cnt++;
			}
		}
		return (cnt == maxCnt);
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
	public Vertex[] getPosition() {
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
		updateCentroid(); // Recalculate centroid
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
	public void setPosition(Vertex[] position) {
		this.position = position;
		updateCentroid();
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
		updateCentroid();
	}

	/**
	 * Recalculates the and centroid. Called automatically when calling <code>setPosition</code>. If
	 * you modify the position array directly, call this afterwards.
	 * 
	 */
	public void updateCentroid() {
		/*
		 * Overridden in triangles class
		 */
	}

	/**
	 * Get edge vectors. Each vector points from one edge point to the next one. For a triangle you
	 * will get 3 edges representing the outer line.
	 * 
	 * @return List of edges
	 */
	public Vector3f[] getEdges() {
		Vector3f e[] = new Vector3f[3];

		for (int j = 0; j < position.length; j++) {
			e[j] = new Vector3f(position[(j + 2) % position.length]);
			e[j].sub(position[(j + 1) % position.length]);
		}
		return e;
	}

}
