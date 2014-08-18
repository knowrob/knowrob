/*******************************************************************************
 * Copyright (c) 2014 Andrei Stoica. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Andrei Stoica - initial API and implementation during Google Summer of Code 2014
 ******************************************************************************/

package edu.tum.cs.vis.model.util;

import javax.vecmath.Vector3f;

/**
 * Class which implements a 3D edge entity. An edge contains of two vertices and of its coordinate
 * vector value.
 * 
 * @author Andrei Stoica
 */
public class Edge {

	/**
	 * Stores the edge coordinates as a 3D floating point vector
	 */
	private final Vector3f	edge			= new Vector3f();

	/**
	 * Stores the reciprocal edge vectorial coordinates vector
	 */
	private final Vector3f	invertedEdge	= new Vector3f();

	/**
	 * Array containing the two vertices that define the edge
	 */
	private final Vertex[]	vertices		= new Vertex[2];

	/**
	 * Boolean field that stores if an edge is sharp or not
	 */
	private boolean			isSharpEdge		= false;

	/**
	 * Default empty constructor for the <tt>Edge</tt> class
	 */
	public Edge() {
		return;
	}

	/**
	 * Constructor for the <tt>Edge</tt> class based on two points. Zero-length edges are allowed,
	 * since they trivially collapse to a point
	 */
	public Edge(Vertex position1, Vertex position2) {
		this.edge.set(position1);
		this.edge.sub(position2);
		this.invertedEdge.negate(edge);
		this.vertices[0] = position1;
		this.vertices[1] = position2;
	}

	/**
	 * Gets the vectorial description of the edge equal to the already computed vectorial difference
	 * between vertex1 and vertex2 associated with the <tt>Edge</tt> object
	 * 
	 * @return the edge 3D vector
	 */
	public Vector3f getEdgeValue() {
		return this.edge;
	}

	/**
	 * Gets the inverted vectorial description of the edge equal to the already computed vectorial
	 * difference between vertex2 and vertex1 associated with the <tt>Edge</tt> object
	 * 
	 * @return the inversed edge 3D vector
	 */
	public Vector3f getInvertedEdgeValue() {
		return this.invertedEdge;
	}

	/**
	 * Gets the array containing the two vertices that define the <tt>Edge</tt> object.
	 * 
	 * @return the array of two vertices of the edge: <tt>vertices[0]</tt> vertex no. 1,
	 *         <tt>vertices[1]</tt> vertex no. 2.
	 */
	public Vertex[] getVerticesOfEdge() {
		return this.vertices;
	}

	/**
	 * Returns whether an edge is sharp or not
	 * 
	 * @return true if edge is sharp and false otherwise
	 */
	public boolean isSharpEdge() {
		return this.isSharpEdge;
	}

	/**
	 * Returns whether a vertex has the same coordinates with the one of the edge vertices. By same
	 * coordinates tolerance spheres of a fixed small radius around the edge vertices are considered
	 * for the comparisons, in order to allow some tolerance given the floating point numbers, see:
	 * {@link edu.tum.cs.vis.model.util.Thresholds#DISTANCE_TOLERANCE} and
	 * {@link edu.tum.cs.vis.model.util.Vertex#sameCoordinates(javax.vecmath.Point3f)}
	 * 
	 * @param v
	 *            vertex to be checked
	 * @return true if vertex is in sphere, false otherwise
	 */
	public boolean hasVertex(Vertex v) {
		if (v != null && (vertices[0].sameCoordinates(v) || vertices[1].sameCoordinates(v))) {
			return true;
		}
		return false;
	}

	/**
	 * Sets the edge vector based on the existing vertices. If one of the vertices is null, nothing
	 * is done. The edge value is defined as <tt>edge := (vertex1 - vertex2)</tt>
	 */
	public void setEdgeValue() {
		if (this.vertices[0] == null || this.vertices[1] == null) {
			return;
		}
		this.edge.set(this.vertices[0]);
		this.edge.sub(this.vertices[1]);
	}

	/**
	 * Sets the inverted edge vector based on the existing vertices. If one of the vertices is null,
	 * nothing is done. The edge value is defined as <tt>edge := (vertex2 - vertex1)</tt>
	 */
	public void setInvertedEdgeValue() {
		this.invertedEdge.negate(this.edge);
	}

	/**
	 * Sets the vertices of the edge object and afterwards computes the edge and inverted edge
	 * vectors.
	 * 
	 * @param position1
	 *            first vertex of the edge
	 * @param position2
	 *            second vertex of the edge
	 */
	public void setVerticesOfEdge(final Vertex position1, final Vertex position2) {
		this.vertices[0] = position1;
		this.vertices[1] = position2;
		this.setEdgeValue();
		this.setInvertedEdgeValue();
	}

	/**
	 * Sets isSharpEdge attribute to a valid boolean value
	 * 
	 * @param value
	 *            value to be set
	 */
	public void isSharpEdge(final boolean value) {
		this.isSharpEdge = value;
	}

	/**
	 * Checks if the edge is equal to the edge passed as argument. Two edges are equal if they have
	 * the same vertices and same edge value
	 * 
	 * @param edge
	 *            edge to be compared with
	 * @return boolean
	 */
	public boolean isEqualTo(Edge edge) {
		if (this.vertices[0].sameCoordinates(edge.vertices[0])
				&& this.vertices[1].sameCoordinates(edge.vertices[1])) {
			return true;
		}
		if (this.vertices[0].sameCoordinates(edge.vertices[1])
				&& this.vertices[1].sameCoordinates(edge.vertices[0])) {
			return true;
		}
		return false;
	}

	/**
	 * Checks if the current instance is has the same direction with the edge passed as argument.
	 * The edges' vectorial description are compared in this sense and depdending on their angle a
	 * decision is taken. Two edges are collinear (have same direction) if the angle between them is
	 * close to 0 or 180 with some tolerance level
	 * {@link edu.tum.cs.vis.model.util.Thresholds#ANGLE_TOLERANCE}
	 * 
	 * @param edge
	 *            to be checked
	 * @return boolean true if collinear / false if not
	 */
	public boolean isCollinear(Edge edge) {
		// compute angle between the two edges in DEG (will result a value in [0,180] DEG)
		float angle = this.edge.angle(edge.edge) * 180.f / (float) Math.PI;
		// if the angle is in [0,angleTolerance] || [180-angleTolerance,180] then edges are
		// collinear
		if (angle <= Thresholds.ANGLE_TOLERANCE || angle >= (180.0f - Thresholds.ANGLE_TOLERANCE)) {
			return true;
		}
		return false;
	}

	/**
	 * Checks if the vertex v belongs to the support line of the edge
	 * 
	 * @param v
	 *            vertex to be checked
	 * @return true if v is on the direction of the edge, false otherwise
	 */
	public boolean containsVertex(Vertex v) {
		if (vertices[0].sameCoordinates(v)) {
			return true;
		}
		if (vertices[1].sameCoordinates(v)) {
			return true;
		}
		Vector3f sideEdge = new Vector3f(v);
		sideEdge.sub(vertices[0]);
		sideEdge.cross(sideEdge, edge);
		if (sideEdge.length() == 0.0f) {
			return true;
		}
		return false;
	}

	/**
	 * Checks if a vertex is inside the edge instance. This is checked taking in to consideration
	 * the two checking spheres of the vertices equality against other vertices. Thus a point is
	 * considered within the edge if it lies around the edge with a small tolerance and it is in
	 * between the boundaries of the two tolerance spheres enclosing the edge vertices.
	 * 
	 * @param v
	 *            vertex
	 * @return true or false
	 */
	public boolean containsVertexInside(Vertex v) {
		Vector3f edgeRef = new Vector3f(invertedEdge);
		// create edge given the vertex to be compared
		Vector3f edgeCmp = new Vector3f(v);
		edgeCmp.sub(vertices[0]);
		if (edgeCmp.length() <= Thresholds.DISTANCE_TOLERANCE) {
			return false;
		}
		// get the angle (in DEG) between the two edges (in [0,180])
		float angle = edgeRef.angle(edgeCmp) * 180.0f / (float) Math.PI;
		// check if the angle is bigger than the tolerance
		if (angle > Thresholds.ANGLE_TOLERANCE && angle < (180.f - Thresholds.ANGLE_TOLERANCE)) {
			return false;
		}
		// now the edges are parallel so v is on the ref edge
		// check if it is inside using dot product
		float dotProd = edgeCmp.dot(edgeRef);
		// the vertex is inside if dotProd in between the edgeRef with the tolerance levels
		// discounted for the edge vertices
		if (dotProd <= Thresholds.DISTANCE_TOLERANCE) {
			return false;
		}
		if (dotProd >= (edge.length() * (edge.length() - Thresholds.DISTANCE_TOLERANCE))) {
			return false;
		}
		return true;
	}

	/**
	 * Checks if the edge passed as an argument is a direct neighbor of the current edge instance. A
	 * direct neighbor is defined as an edge that has the same direction as the edge instance
	 * calling the method, and additionally, also has at least one vertex laying inside the edge
	 * instance (end vertices of this edge are excluded) or both end vertices coincide to the edge
	 * instance calling the method
	 * 
	 * @param nEdge
	 *            edge to be compared to the edge instance callign the method
	 * @return true or false
	 */
	public boolean isDirectNeighbor(Edge nEdge) {
		if (this.isCollinear(nEdge)) {
			if ((this.vertices[0].sameCoordinates(nEdge.vertices[0]) && this.vertices[1]
					.sameCoordinates(nEdge.vertices[1]))
					|| (this.vertices[1].sameCoordinates(nEdge.vertices[0]) && this.vertices[0]
							.sameCoordinates(nEdge.vertices[1]))) {
				return true;
			} else if ((this.containsVertexInside(nEdge.vertices[0]))
					|| (this.containsVertexInside(nEdge.vertices[1]))
					|| (nEdge.containsVertexInside(this.vertices[0]))
					|| (nEdge.containsVertexInside(this.vertices[1]))) {
				return true;
			}
		}
		return false;
	}

	@Override
	public String toString() {
		String print = "Edge: " + this.edge + "\n";
		print = print + "Vertex1: " + this.vertices[0] + ", Vertex2: " + this.vertices[1];
		return print;
	}
}