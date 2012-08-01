/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model;

import java.awt.Color;
import java.util.ArrayList;

import javax.vecmath.Point3f;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

import processing.core.PGraphics;
import edu.tum.cs.vis.model.util.BSphere;
import edu.tum.cs.vis.model.util.Group;
import edu.tum.cs.vis.model.util.Line;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * 
 * Class which represents a CAD model loaded from file. A CAD model consists of triangles and lines
 * which have vertices. These can be grouped hierarchically into subgroups.
 * 
 * 
 * @author Stefan Profanter
 * 
 */
public class Model {

	/**
	 * Absolute file path where the relative paths within the model are based.
	 */
	private String					textureBasePath	= null;

	/**
	 * Main group of the model. Each model should have at least one group.
	 */
	private Group					group;

	/**
	 * List of all vertices in this model
	 */
	private final ArrayList<Vertex>	vertices		= new ArrayList<Vertex>();

	/**
	 * List of all triangles in this model
	 */
	final ArrayList<Triangle>		triangles		= new ArrayList<Triangle>();

	/**
	 * List of all lines in this model
	 */
	private final ArrayList<Line>	lines			= new ArrayList<Line>();

	/**
	 * Minimum bounding sphere of this model. Only set if previously calculated (by Miniball class).
	 * 
	 * @see edu.tum.cs.vis.model.util.algorithm.Miniball
	 */
	private BSphere					boundingSphere	= null;

	/**
	 * Current model scale. Is used to normalize model for further reasoning. getUnscaled methods
	 * use this value to undo scaling for parameters such as height, width, radius, ...
	 */
	private float					scale			= 1;

	/**
	 * @param g
	 *            Graphics context to draw on
	 * @param overrideColor
	 *            Color to override face color of model. Useful if you want to ignore color of model
	 *            and draw whole model in a specific color.
	 */
	public void draw(PGraphics g, Color overrideColor) {
		if (group != null)
			group.draw(g, overrideColor);
	}

	/**
	 * Gets the bounding sphere set by Miniball class.
	 * 
	 * @see edu.tum.cs.vis.model.util.algorithm.Miniball
	 * 
	 * @return the boundingSphere
	 */
	public BSphere getBoundingSphere() {
		return boundingSphere;
	}

	/**
	 * Get main group of model
	 * 
	 * @return the group
	 */
	public Group getGroup() {
		return group;
	}

	/**
	 * Get all lines of model
	 * 
	 * @return the lines
	 */
	public ArrayList<Line> getLines() {
		return lines;
	}

	/**
	 * Gets scale factor of model. Model is scaled (normalized) for reasoning. This value indicates
	 * this scale factor.
	 * 
	 * @return model scale
	 */
	public float getScale() {
		return scale;
	}

	/**
	 * Get texture base path which is the base for all relative texture paths for this model.
	 * 
	 * @return the textureBasePath
	 */
	public String getTextureBasePath() {
		return textureBasePath;
	}

	/**
	 * Get all triangles of model
	 * 
	 * @return the triangles
	 */
	public ArrayList<Triangle> getTriangles() {
		return triangles;
	}

	/**
	 * Unscale the specified float value by dividing <tt>scaled</tt> by <tt>getScale</tt>
	 * 
	 * @param scaled
	 *            scaled value
	 * @return unscaled value
	 */
	public float getUnscaled(float scaled) {
		return scaled * 1f / scale;
	}

	/**
	 * Unscale the specified point coordinates by dividing each coordinate of <tt>corner</tt> by
	 * <tt>getScale</tt>
	 * 
	 * @param corner
	 *            scaled value
	 * @return unscaled value
	 */
	public Tuple3f[] getUnscaled(Point3f[] corner) {
		Tuple3f[] ret = new Point3f[corner.length];
		for (int i = 0; i < corner.length; i++) {
			ret[i] = getUnscaled(corner[i]);
		}
		return ret;
	}

	/**
	 * Unscale the specified float value by dividing <tt>t</tt> by <tt>getScale</tt>
	 * 
	 * @param t
	 *            scaled value
	 * @return unscaled value
	 */
	public Tuple3f getUnscaled(Tuple3f t) {
		Tuple3f tr = new Point3f(t);
		tr.scale(1f / scale);
		return tr;
	}

	/**
	 * Unscale the specified float value by dividing <tt>t</tt> by <tt>getScale</tt>
	 * 
	 * @param t
	 *            scaled value
	 * @return unscaled value
	 */
	public Vector3f getUnscaled(Vector3f t) {
		Vector3f tr = new Vector3f(t);
		tr.scale(1f / scale);
		return tr;
	}

	/**
	 * Get all vertices of model
	 * 
	 * @return the vertices
	 */
	public ArrayList<Vertex> getVertices() {
		return vertices;
	}

	/**
	 * Mirror model on x axis my multiplying each x value of all vertices with -1
	 */
	public void mirrorX() {
		for (Vertex v : vertices) {
			v.x *= (-1);
		}
		group.resetMinMaxValues();

	}

	/**
	 * Normalizes the model. Makes sure that the model fits into a cube with side length of 1. So
	 * each vertex is scaled.
	 */
	public void normalize() {
		float x = group.getMaxX() - group.getMinX();
		float y = group.getMaxY() - group.getMinY();
		float z = group.getMaxZ() - group.getMinZ();

		float max = Math.max(x, Math.max(y, z));

		scale(1f / max);

		scale = 1f / max;
	}

	/**
	 * Scale model by given factor by multiplying all vertices with <tt>factor</tt>. Call setScale
	 * afterwards if you want to set scale factor also for parameters.
	 * 
	 * @param factor
	 *            Factor to scale
	 */
	public void scale(float factor) {

		for (Vertex v : vertices) {
			v.scale(factor);
		}
		for (Triangle t : triangles)
			t.updateCentroid();
		for (Line l : lines)
			l.updateCentroid();
		group.resetMinMaxValues();
	}

	/**
	 * Sets the bounding sphere for the model. Calculated by Miniball class.
	 * 
	 * @see edu.tum.cs.vis.model.util.algorithm.Miniball
	 * 
	 * @param boundingSphere
	 *            the boundingSphere to set
	 */
	public void setBoundingSphere(BSphere boundingSphere) {
		this.boundingSphere = boundingSphere;
	}

	/**
	 * Set main group of model.
	 * 
	 * @param group
	 *            the group to set
	 */
	public void setGroup(Group group) {
		this.group = group;
	}

	/**
	 * Set texture base path for all texture elements of this model.
	 * 
	 * @param textureBasePath
	 *            the textureBasePath to set
	 */
	public void setTextureBasePath(String textureBasePath) {
		this.textureBasePath = textureBasePath;
	}

	/**
	 * Searches for triangles which have exactly the same coordinate points and removes one of them
	 * to avoid two triangles drawn on exactly the same position. Used for mesh reasoning.
	 */
	public void removeDoubleSidedTriangles() {
		for (int i = 0; i < triangles.size(); i++) {
			Triangle t1 = triangles.get(i);
			for (int j = i + 1; j < triangles.size(); j++) {
				Triangle t2 = triangles.get(j);
				int eqCnt = 0;
				for (int k = 0; k < 3; k++) {
					Point3f p1 = t1.getPosition()[k];
					for (Point3f p2 : t2.getPosition()) {
						if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z) {
							eqCnt++;
							break;
						}
					}
					if (eqCnt != k + 1) {
						// break if not enough vertices are equal
						break;
					}
				}
				if (eqCnt == 3) {
					// triangles are the same, so remove j
					triangles.remove(j);
					this.group.removeTriangle(t2);
					j--;
				}
			}

		}
	}

}
