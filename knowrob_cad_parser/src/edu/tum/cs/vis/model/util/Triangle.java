/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.util;

import java.awt.Color;
import java.util.ArrayList;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import org.apache.log4j.Logger;

import processing.core.PConstants;
import processing.core.PGraphics;

/**
 * DrawObject which represents a triangle (object with 3 points).
 * 
 * @author Stefan Profanter
 * 
 */
public class Triangle extends DrawObject {

	/**
	 * auto generated
	 */
	private static final long		serialVersionUID	= -5164768039180386782L;

	/**
	 * log4j logger
	 */
	private static Logger			logger				= Logger.getLogger(Triangle.class);

	/**
	 * Texture-Points
	 */
	protected Point2f				texPosition[];

	/**
	 * Triangles may have normal vector
	 */
	protected Vector3f				normalVector		= null;

	protected Vector3f				cornerarea			= null;

	/**
	 * Centroid of triangle
	 */
	protected Point3f				centroid;

	/**
	 * List of all direct neighbor triangles
	 */
	protected ArrayList<Triangle>	neighbors			= new ArrayList<Triangle>(3);

	/**
	 * Initializes a triangle with given number of edges (Triangle: 3)
	 * 
	 * @param numberOfEdges
	 *            number of edges
	 */
	public Triangle() {
		super(3);
	}

	/**
	 * Add a neighbor to the neighbors list. If list contains already neighbor this method doesn't
	 * add it again. If not, the neighbor will be added and this instance will be added as a
	 * neighbor in the given neighbor instance.
	 * 
	 * @param neighbor
	 *            neighbor to add.
	 */
	public boolean addNeighbor(Triangle neighbor) {
		boolean add = false;
		synchronized (this) {
			if (neighbors.size() >= 3) // for better performance skip if triangle has already all
										// neighbors set
				return false;

			if (neighbors.contains(neighbor))
				return false;

			int eqCnt = 0;

			for (int i = 0; i < 3; i++) {
				if (i == 2 && eqCnt == 0)
					break; // if 2 of 3 points aren't equal, it is no neighbor
				Point3f p1 = position[i];
				for (Point3f p2 : neighbor.position) {
					if (p1 == p2) {
						eqCnt++;
						if (eqCnt == 2) {
							add = true;
							break;
						}
					}

				}
			}

			if (add) {
				synchronized (neighbors) {
					neighbors.add(neighbor);
				}
			}
		}
		if (add) {
			synchronized (neighbor) {
				synchronized (neighbor.neighbors) {
					neighbor.neighbors.add(this);
				}
			}
			return true;
		}
		return false;
	}

	/**
	 * Draw the triangles onto the applet.
	 * 
	 * @param g
	 *            Graphics to draw on
	 * @param overrideColor
	 *            overrides the color to draw
	 */
	public void draw(PGraphics g, Color overrideColor) {
		applyColor(g, overrideColor);
		if (appearance == null || appearance.getImageReference() == null || overrideColor != null) {
			// no texture only color
			g.beginShape(PConstants.TRIANGLES);
			for (int i = 0; i < position.length; i++) {

				if (position[i].color != null)
					g.fill(position[i].color.getRed(), position[i].color.getGreen(),
							position[i].color.getBlue());

				g.vertex(position[i].x, position[i].y, position[i].z);
			}
			g.endShape();

		} else {
			// has texture
			g.beginShape(PConstants.TRIANGLES);
			g.texture(appearance.getImageReference());

			for (int i = 0; i < position.length; i++)
				g.vertex(position[i].x, position[i].y, position[i].z, texPosition[i].x,
						texPosition[i].y);

			g.endShape();

		}

		//TODO check
		/*synchronized (annotations) {

			for (DrawableAnnotation a : annotations)
				a.draw(g);
		}*/

	}

	/**
	 * returns the area of the triangle in 3d space. Unit is the same as you draw the triangle.
	 * 
	 * @return the area in drawing unit.
	 */
	public float getArea() {
		if (position.length == 3) {

			Point3f v = (Point3f) position[1].clone();
			v.sub(position[0]);
			Point3f w = (Point3f) position[2].clone();
			w.sub(position[0]);

			Vector3f v1 = new Vector3f(v);
			Vector3f w1 = new Vector3f(w);

			Vector3f cross = new Vector3f();
			cross.cross(v1, w1);
			return cross.length() / 2f;
		}

		logger.error("getArea not implemented for " + position.length + "-Triangle");
		return 0;
	}

	/**
	 * @return the centroid
	 */
	public Point3f getCentroid() {
		return centroid;
	}

	/**
	 * @return the cornerarea
	 */
	public Vector3f getCornerarea() {
		return cornerarea;
	}

	/**
	 * Get list of all direct neighbor triangles.
	 * 
	 * @return list of triangles
	 * @see Triangle#isNeighbor
	 */
	public ArrayList<Triangle> getNeighbors() {
		return neighbors;
	}

	/**
	 * @return the normalVector
	 */
	public Vector3f getNormalVector() {
		return normalVector;
	}

	/**
	 * @return the texPosition
	 */
	public Point2f[] getTexPosition() {
		return texPosition;
	}

	@Override
	public int hashCode() {
		return position[0].hashCode() ^ position[1].hashCode() ^ position[2].hashCode();
	}

	/**
	 * Checks if this triangle intersects with the given ray (rayStart, rayEnd). Not only the
	 * segment between rayStart and rayEnd is checked but the whole ray from -infinity to +infinity.
	 * 
	 * @param rayStart
	 *            start point of the ray
	 * @param rayEnd
	 *            end point of the ray
	 * @return true if triangle intersects ray
	 */
	public boolean intersectsRay(Point3f rayStart, Point3f rayEnd) {
		return intersectsRay(rayStart, rayEnd, null);
	}

	/**
	 * Checks if this triangle intersects with the given ray (rayStart, rayEnd). Not only the
	 * segment between rayStart and rayEnd is checked but the whole ray from -infinity to +infinity.
	 * 
	 * Copyright 2001, softSurfer (www.softsurfer.com) This code may be freely used and modified for
	 * any purpose providing that this copyright notice is included with it. SoftSurfer makes no
	 * warranty for this code, and cannot be held liable for any real or imagined damage resulting
	 * from its use. Users of this code must verify correctness for their application.
	 * 
	 * @param p1
	 *            start point of the ray
	 * @param p2
	 *            end point of the ray
	 * @param intersectionPoint
	 *            will be set to the intersection point. Set to null to ignore.
	 * @return true if triangle intersects ray
	 */
	public boolean intersectsRay(Point3f p1, Point3f p2, Point3f intersectionPoint) {

		if (position.length != 3) {
			System.out.println("intersectRay not implemented for not triangles!!");
			return false;
		}

		Vector3d u, v, n; // triangle vectors
		Vector3d dir, w0, w; // ray vectors
		double r, a, b; // params to calc ray-plane intersect

		// get triangle edge vectors and plane normal
		u = new Vector3d(position[1]);
		u.sub(new Vector3d(position[0]));
		v = new Vector3d(position[2]);
		v.sub(new Vector3d(position[0]));

		n = new Vector3d();
		n.cross(u, v); // Normal vector
		if (n.equals(new Vector3d(0, 0, 0))) // triangle is degenerate
			return false; // do not deal with this case

		dir = new Vector3d(p2);// ray direction vector
		dir.sub(new Vector3d(p1));

		w0 = new Vector3d(p1);
		w0.sub(new Vector3d(position[0]));

		a = -n.dot(w0);
		b = n.dot(dir);
		// SMALL_NUM 0.00000001 // anything that avoids division overflow
		if (Math.abs(b) < 0.00000001) { // ray is parallel to triangle plane
			if (a == 0) // ray lies in triangle plane
				return false;

			return false; // ray disjoint from plane
		}

		// get intersect point of ray with triangle plane
		r = a / b;
		if (r < 0.0) // ray goes away from triangle
			return false; // => no intersect
		// for a segment, also test if (r > 1.0) => no intersect

		Point3f intersect = intersectionPoint;

		if (intersect == null)
			intersect = new Point3f();

		intersect.set(dir); // intersect point of ray and plane
		intersect.scale((float) r);
		intersect.add(p1);

		// is I inside T?
		double uu, uv, vv, wu, wv, D;
		uu = new Vector3d(u).dot(u);
		uv = new Vector3d(u).dot(v);
		vv = new Vector3d(v).dot(v);
		w = new Vector3d(intersect);
		w.sub(new Vector3d(position[0]));
		wu = new Vector3d(w).dot(u);
		wv = new Vector3d(w).dot(v);
		D = uv * uv - uu * vv;

		// get and test parametric coords
		double s, t;
		s = (uv * wv - vv * wu) / D;
		if (s < 0.0 || s > 1.0) // I is outside T
			return false;
		t = (uv * wu - uu * wv) / D;
		if (t < 0.0 || (s + t) > 1.0) // I is outside T
			return false;

		return true; // I is in T
	}

	/**
	 * Checks if <code>tr</code> is in the list of neighbors. If not the points of <code>this</code>
	 * and <code>tr</code> are compared. If one of them is equal to each other <code>true</code>
	 * will be returned.
	 * 
	 * @param tr
	 *            triangle to check if it is a neighbor
	 * @return true if <code>tr</code> is a neighbor
	 */
	public boolean isNeighbor(Triangle tr) {
		if (neighbors != null && neighbors.contains(tr))
			return true;
		int cnt = 0;
		for (Point3f p1 : position) {
			for (Point3f p2 : tr.position) {
				if (p1 == p2) {
					cnt++;
					if (cnt >= 2)
						return true;
				}
			}
		}
		return false;
	}

	/**
	 * @param cornerarea
	 *            the cornerarea to set
	 */
	public void setCornerarea(Vector3f cornerarea) {
		this.cornerarea = cornerarea;
	}

	/**
	 * Set the list of direct neighbor triangles.
	 * 
	 * @param neighbors
	 *            triangles list.
	 */
	public void setNeighbors(ArrayList<Triangle> neighbors) {
		this.neighbors = neighbors;
	}

	/**
	 * @param normalVector
	 *            * the normalVector to set
	 */
	public void setNormalVector(Vector3f normalVector) {
		this.normalVector = normalVector;
	}

	/**
	 * @param texPosition
	 *            * the texPosition to set
	 */
	public void setTexPosition(Point2f[] texPosition) {
		this.texPosition = texPosition;
	}

	@Override
	public boolean updateNormalVector() {
		normalVector = new Vector3f(0, 0, 0);
		centroid = new Point3f(0, 0, 0);

		for (int i = 0; i < position.length; i++) {
			centroid.add(position[i]);
		}
		centroid.scale(1f / position.length);

		Vector3f a = new Vector3f(position[1]);
		a.sub(position[0]);
		Vector3f b = new Vector3f(position[2]);
		b.sub(position[0]);

		normalVector.cross(a, b);
		if (normalVector.lengthSquared() == 0) {
			normalVector = null;
			return false;
		}
		normalVector.normalize();
		return true;
	}

}
