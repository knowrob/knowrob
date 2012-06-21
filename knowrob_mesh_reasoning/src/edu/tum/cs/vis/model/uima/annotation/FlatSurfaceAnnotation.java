/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation;

import java.awt.Color;
import java.util.ArrayList;

import javax.vecmath.Vector3d;

import edu.tum.cs.vis.model.uima.feature.Area;
import edu.tum.cs.vis.model.uima.feature.Dimension2D;
import edu.tum.cs.vis.model.uima.feature.NormalVector;
import edu.tum.cs.vis.model.util.Triangle;

/**
 * UIMA Annotation for a flat surface.
 * 
 * @author Stefan Profanter
 * @see FlatSurfaceAnalyzer
 * 
 */
/**
 * @author Stefan Profanter
 * 
 */
public class FlatSurfaceAnnotation extends MeshAnnotation {

	/**
	 * auto generated
	 */
	private static final long						serialVersionUID	= 8767464136366533135L;

	/**
	 * Total area of the flat surface
	 */
	private final Area								area				= new Area();

	/**
	 * Dimension2D of the surface in x and y coordinates. So normal vector will be (0,0,1)
	 */
	private final Dimension2D						dimension			= new Dimension2D();
	/**
	 * surface normal
	 */
	private final NormalVector						normalVector		= new NormalVector();

	private final ArrayList<FlatSurfaceAnnotation>	neighbors			= new ArrayList<FlatSurfaceAnnotation>();
	private final ArrayList<Triangle>				neighborTriangles	= new ArrayList<Triangle>();

	/**
	 * Default constructor
	 */
	public FlatSurfaceAnnotation() {
		super(new Color(255, 255, 255, 128));
	}

	public void addNeighbor(FlatSurfaceAnnotation fsa) {
		if (!neighbors.contains(fsa) && isNeighbor(fsa)) {
			neighbors.add(fsa);
			fsa.neighbors.add(this);
		}
	}

	public synchronized void addNeighborTriangles(Triangle p) {
		neighborTriangles.add(p);
	}

	/**
	 * Returns the area of the surface
	 * 
	 * @return the area
	 */
	public Area getArea() {
		return area;
	}

	/**
	 * Returns the dimension of the surface
	 * 
	 * @return the dimension in x and y coordinates. So normal vector will be (0,0,1)
	 */
	public Dimension2D getDimension() {
		return dimension;
	}

	/**
	 * @return the neighbors
	 */
	public ArrayList<FlatSurfaceAnnotation> getNeighbors() {
		return neighbors;
	}

	/**
	 * @return the neighborTriangles
	 */
	public ArrayList<Triangle> getNeighborTriangles() {
		return neighborTriangles;
	}

	/**
	 * Returns the surface normal
	 * 
	 * @return surface normal
	 */
	public Vector3d getNormalVector() {
		return normalVector;
	}

	private boolean isNeighbor(FlatSurfaceAnnotation fsa) {
		for (Triangle p : neighborTriangles) {
			if (fsa.getMesh().getTriangles().contains(p))
				return true;
		}
		return false;
	}

	/**
	 * Updates the members area, dimension, normalVector
	 */
	public void setFeatures() {
		if (mesh.getTriangles().size() == 0)
			return;
		normalVector.set(mesh.getTriangles().get(0).getNormalVector());

		float ar = 0;

		for (Triangle p : mesh.getTriangles())
			ar += p.getArea();
		area.setSquareMM(ar);

		// TODO calculate dimension
	}

}
