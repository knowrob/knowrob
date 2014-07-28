/*******************************************************************************
 * Copyright (c) 2014 Andrei Stoica. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Andrei Stoica - initial API and implementation, Year: 2014
 ******************************************************************************/

package edu.tum.cs.vis.model.util;

import javax.vecmath.Vector3f;

/**
 * Class that implements the functionality of an edge of a triangle
 * 
 * @author Andrei Stoica
 * 
 */
public class Edge {

	/**
	 * Stores the edge as a Vector3f format
	 */
	private final Vector3f	edge			= new Vector3f();

	/**
	 * Stores the reciprocal edge vectorial format
	 */
	private final Vector3f	invertedEdge	= new Vector3f();

	/**
	 * Array containing the two vertices that define the edge
	 */
	private final Vertex[]	vertices		= new Vertex[2];

	/**
	 * Boolean fields that stores if an edge is sharp or not
	 */
	private boolean			isSharpEdge		= false;

	/**
	 * Empty Constructor for the edge
	 */
	public Edge() {
		return;
	}

	/**
	 * Constructor for the edge based on two points
	 */
	public Edge(Vertex position1, Vertex position2) {
		this.edge.set(position1);
		this.edge.sub(position2);
		this.invertedEdge.negate(edge);
		this.vertices[0] = position1;
		this.vertices[1] = position2;
	}

	/**
	 * Get edge method
	 */
	public Vector3f getEdgeValue() {
		return this.edge;
	}

	/**
	 * Get invertedEdge method
	 */
	public Vector3f getInvertedEdgeValue() {
		return this.invertedEdge;
	}

	/**
	 * Get Vertices that define the edge
	 */
	public Vertex[] getVerticesOfEdge() {
		return this.vertices;
	}

	/**
	 * Get isSharpEdge attribute
	 */
	public boolean getIsSharpEdge() {
		return this.isSharpEdge;
	}

	/**
	 * Returns true if edge has vertex v or false otherwise
	 */
	public boolean hasVertex(Vertex v) {
		if (vertices[0].sameCoordinates(v) || vertices[1].sameCoordinates(v)) {
			return true;
		}
		return false;
	}

	/**
	 * Set edge based on existing vertices (if any)
	 */
	public void setEdgeValue() {
		if (this.vertices[0] == null || this.vertices[1] == null) {
			return;
		}
		this.edge.set(this.vertices[0]);
		this.edge.sub(this.vertices[1]);
	}

	/**
	 * Set inverted edge based on existing edge (if any)
	 */
	public void setInvertedEdgeValue() {
		this.invertedEdge.negate(this.edge);
	}

	/**
	 * Set vertices
	 */
	public void setVerticesOfEdge(final Vertex position1, final Vertex position2) {
		this.vertices[0] = position1;
		this.vertices[1] = position2;
		this.setEdgeValue();
		this.setInvertedEdgeValue();
	}

	/**
	 * Set isSharpEdge attribute
	 */
	public void setIsSharpEdge(final boolean value) {
		this.isSharpEdge = value;
	}

	/**
	 * Checks if the edge is equal to the edge passed as argument. Two edges are equal if they have
	 * the same vertices and same value
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
	 * Method that checks if the current instance is collinear to a given edge
	 * 
	 * @param edge
	 * @return boolean true if collinear / false if not
	 */
	public boolean isCollinear(Edge edge) {
		// compute angle between the two edges in DEG (will result a value in [0,180] DEG)
		float angle = this.edge.angle(edge.edge) * 180.f / (float) Math.PI;
		// if the angle is in [0,angleTolerance] || [180-angleTolerance,180] then edges are
		// collinear
		if (angle <= UtilityValues.ANGLE_TOLERANCE
				|| angle >= (180.0f - UtilityValues.ANGLE_TOLERANCE)) {
			return true;
		}
		return false;
	}

	/**
	 * Checks if the vertex v belongs to the edge
	 * 
	 * @param v
	 * @return boolean: true if v is on the direction of the edge, false otherwise
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
	 * Method that checks if a vertex is on the edge instance
	 * 
	 * @param v
	 *            vertex
	 * @return boolean true or false
	 */
	public boolean containsInsideVertex(Vertex v) {
		Vector3f edgeRef = new Vector3f(invertedEdge);
		// create edge given the vertex to be compared
		Vector3f edgeCmp = new Vector3f(v);
		edgeCmp.sub(vertices[0]);
		if (edgeCmp.length() <= UtilityValues.DISTANCE_TOL) {
			return false;
		}
		// get the angle (in DEG) between the two edges (in [0,180])
		float angle = edgeRef.angle(edgeCmp) * 180.0f / (float) Math.PI;
		// check if the angle is bigger than the tolerance
		if (angle > UtilityValues.ANGLE_TOLERANCE
				&& angle < (180.f - UtilityValues.ANGLE_TOLERANCE)) {
			return false;
		}
		// now the edges are parallel so v is on the ref edge
		// check if it is inside using dot product
		float dotProd = edgeCmp.dot(edgeRef);
		// the vertex is inside if dotProd in between the edgeRef with the tolerance levels
		// discounted for the edge vertices
		if (dotProd <= UtilityValues.DISTANCE_TOL) {
			return false;
		}
		if (dotProd >= (edge.length() * (edge.length() - UtilityValues.DISTANCE_TOL))) {
			return false;
		}
		return true;
	}

	/**
	 * Method that checks if the edge passed as an argument is a direct neighbor of the current edge
	 * instance. A direct neighbor is defined as an edge that has the same direction as the edge
	 * instance calling the method, and additionally, also has at least one vertex laying inside the
	 * edge instance (end vertices of this edge are excluded) or both end vertices coincide to the
	 * edge instance calling the method
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
			} else if ((this.containsInsideVertex(nEdge.vertices[0]))
					|| (this.containsInsideVertex(nEdge.vertices[1]))
					|| (nEdge.containsInsideVertex(this.vertices[0]))
					|| (nEdge.containsInsideVertex(this.vertices[1]))) {
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