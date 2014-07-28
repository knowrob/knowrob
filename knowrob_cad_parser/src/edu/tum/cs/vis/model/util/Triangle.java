/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.util;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
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
	private static final long	serialVersionUID			= -5164768039180386782L;

	/**
	 * log4j logger
	 */
	private static Logger		logger						= Logger.getLogger(Triangle.class);

	/**
	 * Texture-Points
	 */
	protected Point2f			texPosition[];

	/**
	 * Triangles may have normal vector
	 */
	protected Vector3f			normalVector				= null;

	/**
	 * Voronoi area of triangle
	 */
	protected Vector3f			cornerarea					= null;

	/**
	 * Centroid of triangle
	 */
	protected Point3f			centroid;

	/**
	 * List of all direct neighbor triangles
	 */
	protected Set<Triangle>		neighbors					= new HashSet<Triangle>(3);

	/**
	 * Array containing the three edges that define the triangle as returned from the getEdges
	 * inherited routine from DrawObject
	 */
	protected Edge[]			edges						= new Edge[3];

	/**
	 * Stores if a triangle contains three "sharp" vertices
	 */
	private boolean				isSharpTriangle				= false;

	/**
	 * Stores if a triangle is a "seed triangle" as described in
	 * 
	 * Guillaume Lavoue, Florent Dupont, Atilla Baskurt, "A new CAD mesh segmentation method, based
	 * on curvature tensor analysis", Computer-Aided Design 37(2005) 975-987
	 */
	private boolean				isSeedTriangle				= false;

	/**
	 * Stores the region label which the triangle belongs to
	 */
	private int					regionLabel					= -1;

	/**
	 * Stores the Min and Max Curvature associated to the triangle in the case the triangle is a
	 * "seed triangle" as defined in the paper
	 * 
	 * Guillaume Lavoue, Florent Dupont, Atilla Baskurt, "A new CAD mesh segmentation method, based
	 * on curvature tensor analysis", Computer-Aided Design 37(2005) 975-987.
	 * 
	 * KMin is stored on position 0 and KMax is stored on position 1.
	 * 
	 */
	private final float[]		curvatureMinMaxValue		= { 0.0f, 0.0f };

	/**
	 * Flag to mark whether the curvatures Min Max values have been modified from default
	 */
	private boolean				isCurvatureMinMaxValueInit	= false;

	public boolean				isVisited					= false;

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
	 * Add an isolated neighbor to the neighbors list. If list contains already neighbor this
	 * doesn't add again. If not, the neighbor will be added and this instance will be added as a
	 * neighbor in the given neighbor instance.
	 * 
	 * Use with care to add just created triangles for the sharp detection process!
	 * 
	 * @param neighbor
	 *            neighbor to add
	 */
	public void addNeighbor(Triangle neighbor) {
		synchronized (neighbors) {
			neighbors.add(neighbor);
		}
		synchronized (neighbor.neighbors) {
			neighbor.neighbors.add(this);
		}

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
	 * Get neighbor of a certain edge from the CAD model.
	 * 
	 * @return triangle
	 */
	public List<Triangle> getNeighborsOfEdge(final Edge edge) {
		List<Triangle> toAdd = new ArrayList<Triangle>();
		for (Triangle t : neighbors) {
			Edge[] tEdges = t.getEdges();
			for (int i = 0; i < tEdges.length; ++i) {
				if (edge.isDirectNeighbor(tEdges[i])) {
					toAdd.add(t);
				}
			}
		}
		return toAdd;
	}

	/**
	 * Get common edge of two different triangles
	 * 
	 * @param triangle
	 * @return edge
	 */
	public Edge getCommonEdge(final Triangle triangle) {
		if (neighbors.contains(triangle)) {
			Edge[] triangleEdges = triangle.getEdges();
			for (int i = 0; i < edges.length; ++i) {
				for (int j = 0; j < triangleEdges.length; ++j) {
					if (edges[i].isDirectNeighbor(triangleEdges[j])) {
						return edges[i];
					}
				}
			}
		}
		return null;
	}

	/**
	 * Get an array containing the 3 edges that define the triangle as returned by the
	 * DrawObject.getEdges() method => need to call updateEdges() method first to initialize them
	 * 
	 * @return edges
	 */
	@Override
	public Edge[] getEdges() {
		return edges;
	}

	/**
	 * Get an array of sharp edges in the order given by the edges array
	 * 
	 * NOTE: This needs to be called after the sharp edge detection processing in order to deliver
	 * the expected results!
	 * 
	 * @return array of sharp edges
	 */
	public Edge[] getSharpEdges() {
		int numSharpEdges = 0;
		for (int i = 0; i < edges.length; ++i) {
			if (edges[i].getIsSharpEdge()) {
				numSharpEdges++;
			}
		}
		if (numSharpEdges == 3) {
			return edges;
		}
		Edge[] sharpEdges = new Edge[numSharpEdges];
		numSharpEdges = 0;
		for (int i = 0; i < edges.length; ++i) {
			if (edges[i].getIsSharpEdge()) {
				sharpEdges[numSharpEdges++] = edges[i];
			}
		}
		return sharpEdges;
	}

	/**
	 * Get an array of non-sharp edges in the order given by the edges array.
	 * 
	 * NOTE: This needs to be called after sharp edge detection processing in order to deliver the
	 * expected results!
	 * 
	 * @return array of non-sharp edges
	 */
	public Edge[] getNonSharpEdges() {
		int numNonSharpEdges = 0;
		for (int i = 0; i < edges.length; ++i) {
			if (!edges[i].getIsSharpEdge()) {
				numNonSharpEdges++;
			}
		}
		if (numNonSharpEdges == 3) {
			return edges;
		}
		Edge[] nonSharpEdges = new Edge[numNonSharpEdges];
		if (numNonSharpEdges == 0) {
			return nonSharpEdges;
		}
		numNonSharpEdges = 0;
		for (int i = 0; i < edges.length; ++i) {
			if (!edges[i].getIsSharpEdge()) {
				nonSharpEdges[numNonSharpEdges++] = edges[i];
			}
		}
		return nonSharpEdges;
	}

	/**
	 * Determines the opposite vertex from an edge passed as a parameter. The edge needs to be a
	 * neighboring edge. This is not checked! Handle with care!
	 * 
	 * @param edge
	 * 
	 * @return opposite vertex of the edge
	 */
	public Vertex getOppositeVertexFromEdge(Edge edge) {
		boolean flagNotDirectNeighbor = true;
		// first get the common edge section belonging to the triangle
		for (int i = 0; i < edges.length; ++i) {
			if (edges[i].isDirectNeighbor(edge)) {
				edge = edges[i];
				flagNotDirectNeighbor = false;
			}
		}
		if (flagNotDirectNeighbor) {
			return null;
		}
		// determine opposite vertex
		for (int i = 0; i < position.length; ++i) {
			if (!edge.hasVertex(position[i])) {
				return position[i];
			}
		}
		logger.debug("bad tesselation @ " + this.toString());
		return null;
	}

	/**
	 * Gets the curvature values for the current triangle.
	 * 
	 * To be called only if before the method isSeedTriangle has been called
	 */
	public float[] getCurvatureValues() {
		return curvatureMinMaxValue;
	}

	/**
	 * Gets the value of the field isCurvatureMinMaxValueInit
	 */
	public boolean getIsCurvatureMinMaxValueInit() {
		return isCurvatureMinMaxValueInit;
	}

	/**
	 * Adds a sharp edge to the triangle if this belongs to the triangle. In this case, it also
	 * marks the edge as being sharp
	 */
	public void addSharpEdge(Edge edge) {
		for (int i = 0; i < edges.length; ++i) {
			if (edges[i].isDirectNeighbor(edge)) {
				edges[i].setIsSharpEdge(true);
				return;
			}
		}
	}

	/**
	 * Gets the region label property of the current Triangle object
	 */
	public int getRegionLabel() {
		return regionLabel;
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
		// double dot = this.getNormalVector().dot(t.getNormalVector());
		// dot = Math.max(-1, dot);
		// dot = Math.min(1, dot);
		// return Math.acos(dot);
		return this.getNormalVector().angle(t.getNormalVector());
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
		for (int i = 0; i < position.length; i++) {
			if (i == 2 && eqCnt == 0)
				break; // if 2 of 3 points aren't equal, it is no neighbor
			Vertex p1 = position[i];
			for (Vertex p2 : tr.position) {
				if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z) {
					eqCnt++;
					if (eqCnt == 2) {
						isNeighbor = true; // two common vertices, so neighbors
					} else if (eqCnt == 3) {
						return false; // similar triangles back to back
					}
					break;
				}
			}
		}
		// also mind the inexact triangle hits (vertices are not the same)
		if (isNeighbor == false) {
			eqCnt = 0;
			Edge[] trEdges = tr.getEdges();
			for (int i = 0; i < edges.length; ++i) {
				for (int j = 0; j < trEdges.length; ++j) {
					if (edges[i].isDirectNeighbor(trEdges[j])) {
						eqCnt++;
					}
				}
			}
			// if one hit only happened the triangles are neighbors
			if (eqCnt == 1) {
				return true;
			}
			// otherwise they are not neighbors at all or just the same or back-faced
			return false;
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

	/**
	 * Checks if the triangle contains or not the edge passed as argument
	 * 
	 * @param edge
	 *            edge to be checked if in triangle
	 */
	public boolean containsEdge(Edge edge) {
		for (int i = 0; i < edges.length; ++i) {
			if (edges[i].isEqualTo(edge)) {
				return true;
			}
		}
		return false;
	}

	/**
	 * States whether or not the triangle has 3 sharp vertices
	 */
	public boolean isSharpTriangle() {
		return isSharpTriangle;
	}

	/**
	 * Checks if the triangle has indeed 3 sharp vertices and then sets its property accordingly
	 */
	public boolean checkIsSharpTriangle() {
		if (position[0].isSharpVertex() && position[1].isSharpVertex()
				&& position[2].isSharpVertex()) {
			this.isSharpTriangle = true;
			return true;
		}
		return false;
	}

	/**
	 * Sets the region label of the triangle
	 */
	public void setRegionLabel(final int newLabel) {
		this.regionLabel = newLabel;
	}

	/**
	 * Helper method that returns the number of sharp vertices of the current triangle object, as
	 * defined in
	 * 
	 * Guillaume Lavoue, Florent Dupont, Atilla Baskurt, "A new CAD mesh segmentation method, based
	 * on curvature tensor analysis", Computer-Aided Design 37(2005) 975-987.
	 * 
	 */
	public int numOfSharpVertices() {
		int cont = 0;
		for (int i = 0; i < position.length; ++i) {
			if (position[i].isSharpVertex()) {
				cont++;
			}
		}
		return cont;
	}

	/**
	 * Updates the edges array content using the Vertices of the Triangle object
	 */
	public void updateEdges() {
		for (int j = 0; j < position.length; j++) {
			edges[j] = new Edge(position[(j + 2) % position.length], position[(j + 1)
					% position.length]);
		}
	}

	/**
	 * Checks if the triangle is a "seed triangle" as described in the paper:
	 * 
	 * Guillaume Lavoue, Florent Dupont, Atilla Baskurt, "A new CAD mesh segmentation method, based
	 * on curvature tensor analysis", Computer-Aided Design 37(2005) 975-987.
	 * 
	 * and if this is true it also sets the triangle labeled KMin and KMax values as described in
	 * the same paper
	 */
	public boolean updateIsSeedTriangle() {
		int numOfSharpVertices = this.numOfSharpVertices();
		if (numOfSharpVertices == 0) {
			if (((position[0].getClusterCurvatureVal()[0] == position[1].getClusterCurvatureVal()[0]) && (position[1]
					.getClusterCurvatureVal()[0] == position[2].getClusterCurvatureVal()[0]))
					&& ((position[0].getClusterCurvatureVal()[1] == position[1]
							.getClusterCurvatureVal()[1]) && (position[1].getClusterCurvatureVal()[1] == position[2]
							.getClusterCurvatureVal()[1]))) {
				// if (position[0].getClusterCurvatureVal()[0] == 0
				// && position[0].getClusterCurvatureVal()[1] == 0) {
				// System.out.println("yes");
				// }
				setCurvatureLevels(position[0].getClusterCurvatureVal()[0],
						position[0].getClusterCurvatureVal()[1]);
				this.isSeedTriangle = true;
				return true;
			}
			this.isSeedTriangle = false;
			return false;
		} else if (numOfSharpVertices == 1) {
			Vertex[] v = new Vertex[2];
			int cont = 0;
			if (!position[0].isSharpVertex()) {
				v[cont] = position[0];
				++cont;
			}
			if (!position[1].isSharpVertex()) {
				v[cont] = position[1];
				++cont;
			}
			if (!position[2].isSharpVertex() && cont < 2) {
				v[cont] = position[2];
			}
			if ((v[0].getClusterCurvatureVal()[0] == v[1].getClusterCurvatureVal()[0])
					&& v[0].getClusterCurvatureVal()[1] == v[1].getClusterCurvatureVal()[1]) {
				setCurvatureLevels(v[0].getClusterCurvatureVal()[0],
						v[0].getClusterCurvatureVal()[1]);
				this.isSeedTriangle = true;
				return true;
			}
			this.isSeedTriangle = false;
			return false;
		} else if (numOfSharpVertices == 2) {
			for (int i = 0; i < position.length; ++i) {
				if (position[i].isSharpVertex()) {
					setCurvatureLevels(position[i].getClusterCurvatureVal()[0],
							position[i].getClusterCurvatureVal()[1]);
					this.isSeedTriangle = true;
					return true;
				}
			}
			this.isSeedTriangle = false;
			return false;
		}
		this.isSeedTriangle = false;
		return false;
	}

	/**
	 * Getter for the isSeedTriangle property. To be called after the update method has been called
	 */
	public boolean isSeedTriangle() {
		return isSeedTriangle;
	}

	/**
	 * Sets the curvature values for the current triangle when this is checked as being a "seed"
	 * triangle, according to the paper
	 * 
	 * Guillaume Lavoue, Florent Dupont, Atilla Baskurt, "A new CAD mesh segmentation method, based
	 * on curvature tensor analysis", Computer-Aided Design 37(2005) 975-987.
	 * 
	 */
	public void setCurvatureLevels(final float minCurvatureLevel, final float maxCurvatureLevel) {
		this.isCurvatureMinMaxValueInit = true;
		this.curvatureMinMaxValue[0] = minCurvatureLevel;
		this.curvatureMinMaxValue[1] = maxCurvatureLevel;
	}

	@Override
	public void updateCentroid() {
		centroid = new Point3f(0, 0, 0);

		for (int i = 0; i < position.length; i++) {
			centroid.add(position[i]);
		}
		centroid.scale(1f / position.length);
	}

	@Override
	public String toString() {
		String print = "Triangle: " + System.identityHashCode(this) + "\n";
		print = print + "V1: " + position[0] + " V2: " + position[1] + " V3: " + position[2] + "\n";
		print = print + "isSeedTriangle: " + isSeedTriangle + "\n";
		if (isSeedTriangle || (curvatureMinMaxValue[0] != 0.0f && curvatureMinMaxValue[1] != 0.0f)) {
			print = print + "Region: " + regionLabel + "\n";
			print = print + "KMin: " + curvatureMinMaxValue[0] + "KMax: " + curvatureMinMaxValue[1]
					+ "\n";
		}
		return print;
	}
}
