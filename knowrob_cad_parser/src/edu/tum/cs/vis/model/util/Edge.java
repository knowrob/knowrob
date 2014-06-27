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

	@Override
	public String toString() {
		String print = "Edge: " + this.edge + "\n";
		print = print + "Vertex1: " + this.vertices[0] + ", Vertex2: " + this.vertices[1];
		return print;
	}
}