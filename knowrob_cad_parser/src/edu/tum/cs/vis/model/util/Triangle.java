/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.util;

import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.locks.Lock;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import javax.vecmath.Tuple3f;
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
	private static final long	serialVersionUID	= -5164768039180386782L;

	/**
	 * log4j logger
	 */
	private static Logger		logger				= Logger.getLogger(Triangle.class);

	/**
	 * Texture-Points
	 */
	protected Point2f			texPosition[];

	/**
	 * Triangles may have normal vector
	 */
	protected Vector3f			normalVector		= null;

	/**
	 * Voronoi area of triangle
	 */
	protected Vector3f			cornerarea			= null;

	/**
	 * Centroid of triangle
	 */
	protected Point3f			centroid;

	/**
	 * List of all direct neighbor triangles
	 */
	protected Set<Triangle>		neighbors			= new HashSet<Triangle>(3);

	/**
	 * Initializes a triangle with given number of edges (Triangle: 3)
	 * 
	 */
	public Triangle() {
		super(3);
	}

	/**
	 * Create new triangle by the given three corner points.
	 * 
	 * @param p1
	 *            corner 1
	 * @param p2
	 *            corner 2
	 * @param p3
	 *            corner 3
	 */
	public Triangle(Tuple3f p1, Tuple3f p2, Tuple3f p3) {
		this(new Vertex(p1), new Vertex(p2), new Vertex(p3));
	}

	/**
	 * Create a new triangle by the given three corner points and their normal vertices.
	 * 
	 * @param p1
	 *            corner 1
	 * @param normP1
	 *            normal vertex at corner 1
	 * @param p2
	 *            corner 2
	 * @param normP2
	 *            normal vertex at corner 2
	 * @param p3
	 *            corner 3
	 * @param normP3
	 *            normal vertex at corner 3
	 */
	public Triangle(Tuple3f p1, Vector3f normP1, Tuple3f p2, Vector3f normP2, Tuple3f p3,
			Vector3f normP3) {
		super(3);
		position[0] = new Vertex(p1, normP1);
		position[1] = new Vertex(p2, normP2);
		position[2] = new Vertex(p3, normP3);
	}

	/**
	 * Create new triangle by the given three corner points.
	 * 
	 * @param p1
	 *            corner 1
	 * @param p2
	 *            corner 2
	 * @param p3
	 *            corner 3
	 */
	public Triangle(Vertex p1, Vertex p2, Vertex p3) {
		super(3);
		position[0] = p1;
		position[1] = p2;
		position[2] = p3;
	}

	/**
	 * Add a neighbor to the neighbors list. If list contains already neighbor this method doesn't
	 * add it again. If not, the neighbor will be added and this instance will be added as a
	 * neighbor in the given neighbor instance.
	 * 
	 * @param neighbor
	 *            neighbor to add.
	 * @param lock
	 *            Mutex lock to avoid multithreading issues. This object should be the same for all
	 *            calling threads. Used to synchronizes adding a neighbor to the triangle
	 * @return true if <tt>neighbor</tt> is really a neighbor of this triangle and it was
	 *         successfully added.
	 * 
	 * 
	 */
	public boolean addNeighbor(Triangle neighbor, Lock lock) {
		boolean add = false;

		if (neighbors.size() >= 3) // for better performance skip if triangle has already all
									// neighbors set
			return false;

		add = isDirectNeighbor(neighbor);

		if (add) {
			if (lock != null)
				lock.lock();
			// to make sure neighbors is not cached
			synchronized (neighbors) {
				neighbors.add(neighbor);
			}
			synchronized (neighbor.neighbors) {
				neighbor.neighbors.add(this);
			}
			if (lock != null)
				lock.unlock();
		}
		return add;
	}

	/**
	 * Removes the given triangle from the neighbor relation of this triangle. This triangle is also
	 * removed from <tt>n</tt>'s neighbor list.
	 * 
	 * @param n
	 *            neighbor to remove
	 */
	public void removeNeighbor(Triangle n) {
		if (neighbors != null) {
			synchronized (neighbors) {
				neighbors.remove(n);
			}
		}
		if (n.neighbors != null) {
			synchronized (n.neighbors) {
				n.neighbors.remove(this);
			}
		}
	}

	/**
	 * Draw the triangles onto the applet.
	 * 
	 * @param g
	 *            Graphics to draw on
	 * @param drawSettings
	 *            override the draw color, texture (and other settings). Draw whole object in the
	 *            given color if != null
	 */
	public void draw(PGraphics g, DrawSettings drawSettings) {
		applyColor(g, drawSettings);

		if (drawSettings != null && drawSettings.drawType == DrawType.POINTS) {

			for (int i = 0; i < position.length; i++) {
				if (position[i].overrideColor != null) {
					g.stroke(position[i].overrideColor.getRed(),
							position[i].overrideColor.getGreen(),
							position[i].overrideColor.getBlue());
					g.noFill();
				} else if (drawSettings.getOverrideColor() == null && position[i].color != null) {
					g.stroke(position[i].color.getRed(), position[i].color.getGreen(),
							position[i].color.getBlue());
					g.noFill();
				}
				g.point(position[i].x, position[i].y, position[i].z);
			}
		} else {
			g.beginShape(PConstants.TRIANGLES);
			if (appearance == null || appearance.getImageReference() == null /*no appearance*/
					|| drawSettings.getOverrideColor() != null /*override*/
					|| position[0].overrideColor != null /* points have colors */) {
				// no texture only color

				for (int i = 0; i < position.length; i++) {

					if (drawSettings != null && drawSettings.drawType == DrawType.LINES) {
						if (position[i].overrideColor != null) {
							g.stroke(position[i].overrideColor.getRed(),
									position[i].overrideColor.getGreen(),
									position[i].overrideColor.getBlue());
							g.noFill();
						} else if (drawSettings.getOverrideColor() == null
								&& position[i].color != null) {
							g.stroke(position[i].color.getRed(), position[i].color.getGreen(),
									position[i].color.getBlue());
							g.noFill();
						}
					} else {
						if (position[i].overrideColor != null) {
							g.fill(position[i].overrideColor.getRed(),
									position[i].overrideColor.getGreen(),
									position[i].overrideColor.getBlue());
						} else if ((drawSettings == null || drawSettings.getOverrideColor() == null)
								&& position[i].color != null) {
							g.fill(position[i].color.getRed(), position[i].color.getGreen(),
									position[i].color.getBlue());
						}
					}

					if (position[i].getNormalVector() != null)
						g.normal(position[i].getNormalVector().x, position[i].getNormalVector().y,
								position[i].getNormalVector().z);
					g.vertex(position[i].x, position[i].y, position[i].z);
				}

			} else {
				// has texture
				g.texture(appearance.getImageReference());

				for (int i = 0; i < position.length; i++) {

					g.vertex(position[i].x, position[i].y, position[i].z, texPosition[i].x,
							texPosition[i].y);

				}

			}
			g.endShape();
		}
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
	 * Get centroid of triangle
	 * 
	 * @return the centroid
	 */
	public Point3f getCentroid() {
		return centroid;
	}

	/**
	 * Get voronoi area of triangle
	 * 
	 * @return the cornerarea
	 */
	public Vector3f getCornerarea() {
		return cornerarea;
	}

	/**
	 * Get list of all direct neighbor triangles.
	 * 
	 * @return list of triangles
	 */
	public Set<Triangle> getNeighbors() {
		return neighbors;
	}

	/**
	 * Calculates the angle between surface normal of this triangle and the given one. Angle is
	 * between 0 and PI (180°).
	 * 
	 * @param t
	 *            counterpart triangle
	 * @return angle in radiant between 0 and PI
	 */
	public double getDihedralAngle(Triangle t) {
		double dot = this.getNormalVector().dot(t.getNormalVector());
		dot = Math.max(-1, dot);
		dot = Math.min(1, dot);
		return Math.acos(dot);
	}

	/**
	 * get normal vector of triangle
	 * 
	 * @return the normalVector
	 */
	public Vector3f getNormalVector() {
		return normalVector;
	}

	/**
	 * Get position array for texture
	 * 
	 * @return the texPosition
	 */
	public Point2f[] getTexPosition() {
		return texPosition;
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
	 * Calculate and set normal vector for triangle
	 * 
	 * @return true if vector successfully calculated
	 */
	public boolean calculateNormalVector() {
		// Calculate normal vector for triangle
		Vector3f avgVertexNorm = new Vector3f();
		for (int i = 0; i < 3; i++) {
			if (position[i].getNormalVector() == null) {
				avgVertexNorm = null;
				break;
			}
			avgVertexNorm.add(position[i].getNormalVector());
		}
		if (avgVertexNorm != null) {
			avgVertexNorm.scale(1f / 3f);
		}

		Vector3f a = new Vector3f(position[0]);
		a.sub(position[1]);
		Vector3f b = new Vector3f(position[1]);
		b.sub(position[2]);

		/*Vector3f a = new Vector3f(position[1]);
		a.sub(position[0]);
		Vector3f b = new Vector3f(position[2]);
		b.sub(position[0]);*/
		Vector3f norm = new Vector3f();
		norm.cross(a, b);
		if (norm.lengthSquared() == 0) {
			return false;
		}
		norm.normalize();

		if (avgVertexNorm != null && avgVertexNorm.dot(norm) < 0)
			norm.scale(-1f);

		this.normalVector = norm;
		return true;
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

		// if (r < 0.0) // ray goes away from triangle
		// return false; // => no intersect
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
	 * Checks if <code>tr</code> is a direct neighbor which means that at least 2 vertices of both
	 * triangles must be at the same coordinate.
	 * 
	 * @param tr
	 *            triangle to check if it is a neighbor
	 * @return true if <code>tr</code> is a neighbor
	 */
	public boolean isDirectNeighbor(Triangle tr) {

		int eqCnt = 0;
		boolean isNeighbor = false;

		for (int i = 0; i < 3; i++) {
			if (i == 2 && eqCnt == 0)
				break; // if 2 of 3 points aren't equal, it is no neighbor
			Point3f p1 = position[i];
			for (Point3f p2 : tr.position) {
				if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z) {
					eqCnt++;
					if (eqCnt == 2) {
						isNeighbor = true;
					} else if (eqCnt == 3) {
						// if triangle has same position but is backface
						return false;
					}
					break;
				}

			}
		}
		return isNeighbor;
	}

	/**
	 * Checks if <code>tr</code> is an adjacent neighbor which means that at least 2 vertices of
	 * both triangles must be not only the same coordinates but also the same object.
	 * 
	 * @param tr
	 *            triangle to check if it is a neighbor
	 * @return true if <code>tr</code> is a neighbor
	 */
	public boolean isAdjacentNeighbor(Triangle tr) {

		int eqCnt = 0;
		boolean isNeighbor = false;

		for (int i = 0; i < 3; i++) {
			if (i == 2 && eqCnt == 0)
				break; // if 2 of 3 points aren't equal, it is no neighbor
			Point3f p1 = position[i];
			for (Point3f p2 : tr.position) {
				if (p1 == p2) {
					eqCnt++;
					if (eqCnt == 2) {
						isNeighbor = true;
					}
					if (eqCnt == 3) {
						// if triangle has same position but is backface
						isNeighbor = false;
					}
					break;
				}

			}
		}
		return isNeighbor;
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
	public void setNeighbors(Set<Triangle> neighbors) {
		this.neighbors = neighbors;
	}

	/**
	 * Set normal vector of triangle
	 * 
	 * @param normalVector
	 *            the normalVector to set
	 */
	public void setNormalVector(Vector3f normalVector) {
		this.normalVector = normalVector;
	}

	/**
	 * set texture position array for texture of triangle
	 * 
	 * @param texPosition
	 *            * the texPosition to set
	 */
	public void setTexPosition(Point2f[] texPosition) {
		this.texPosition = texPosition;
	}

	@Override
	public void updateCentroid() {
		centroid = new Point3f(0, 0, 0);

		for (int i = 0; i < position.length; i++) {
			centroid.add(position[i]);
		}
		centroid.scale(1f / position.length);
	}

}
