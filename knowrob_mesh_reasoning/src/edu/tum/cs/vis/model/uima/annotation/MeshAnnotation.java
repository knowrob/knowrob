/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation;

import java.awt.Color;
import java.util.HashSet;

import processing.core.PGraphics;
import edu.tum.cs.vis.model.Model;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Mesh;
import edu.tum.cs.vis.model.util.Triangle;

/**
 * Base class for all mesh annotations.
 * 
 * @author Stefan Profanter
 * 
 */
public abstract class MeshAnnotation<S extends MeshAnnotation> extends DrawableAnnotation {

	private final Class<S>		clazz;

	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= 1222063742441634463L;

	/**
	 * The annotation color for this type of annotation.
	 */
	protected Color				annotationColor;
	/**
	 * A random annotation color. Each annotation gets also a random color.
	 */
	private final Color			randomAnnotationColor;

	/**
	 * Use random color for drawing the annotation
	 */
	protected boolean			useRandomColor		= false;

	/**
	 * Mesh which contains the referenced Triangles for which this annotation stands.
	 */
	protected Mesh				mesh				= new Mesh();

	protected Model				model;

	/**
	 * Default constructor. Sets the annotation color. Each type of annotation should have a
	 * different color.
	 * 
	 * @param annotationColor
	 *            The annotation color for this type of annotation
	 */
	public MeshAnnotation(Class<S> clazz, Model model, final Color annotationColor) {
		super();
		this.clazz = clazz;
		randomAnnotationColor = new Color((int) (Math.random() * 255), (int) (Math.random() * 255),
				(int) (Math.random() * 255));
		this.annotationColor = annotationColor;
		this.model = model;
	}

	@Override
	protected void drawAnnotation(PGraphics g) {
		mesh.drawLines(g, getDrawColor());
		mesh.drawTriangles(g, getDrawColor());
	}

	/**
	 * Returns the color for drawing this annotation. If useRandomColor is true, the random color
	 * will be returned.
	 * 
	 * @return randomAnnotationColor if useRandomColor. Otherwise: annotationColor
	 */
	public Color getDrawColor() {
		if (useRandomColor)
			return randomAnnotationColor;

		return annotationColor;
	}

	/**
	 * @return the mesh
	 */
	public Mesh getMesh() {
		return mesh;
	}

	public HashSet<S> getNeighborAnnotations(MeshCas cas) {
		return getNeighborAnnotations(cas, clazz);
	}

	public <T extends MeshAnnotation> HashSet<T> getNeighborAnnotations(MeshCas cas,
			Class<T> parClazz) {
		HashSet<T> annotations = new HashSet<T>();
		for (Triangle t : getMesh().getTriangles()) {
			annotations.addAll(getNeighborAnnotationsForTriangle(cas, parClazz, t));
		}
		if (annotations.contains(this))
			annotations.remove(this);
		return annotations;
	}

	public <T extends MeshAnnotation> HashSet<T> getNeighborAnnotationsForTriangle(MeshCas cas,
			Class<T> parClazz, Triangle t) {

		HashSet<T> annotations = new HashSet<T>();
		/* 
		 * Triangle has max 3 neighbors
		 * First iteration set a1 to annotation of neighbor 1, second set a2 to annotation of neighbor 2.
		 * On third iteration 
		 */

		// Check all neighbors of the triangle which annotation they have
		for (Triangle neig : t.getNeighbors()) {
			// neighbor is in same annotation, skip
			if (getMesh().getTriangles().contains(neig))
				continue;

			// Get annotation of triangle
			T ma = cas.findAnnotation(parClazz, neig);
			if (ma != null)
				annotations.add(ma);
		}
		return annotations;
	}

	/**
	 * @return the useRandomColor
	 */
	public boolean isUseRandomColor() {
		return useRandomColor;
	}

	/**
	 * Checks if this annotation includes the triangle <code>p</code>.
	 * 
	 * @param p
	 *            triangle to check for
	 * @return true if annotation includes triangle
	 */
	public boolean meshContainsTriangle(final Triangle p) {
		return mesh.getTriangles().contains(p);
	}

	/**
	 * @param mesh
	 *            the mesh to set
	 */
	public void setMesh(Mesh mesh) {
		this.mesh = mesh;
	}

	/**
	 * @param useRandomColor
	 *            the useRandomColor to set
	 */
	public void setUseRandomColor(boolean useRandomColor) {
		this.useRandomColor = useRandomColor;
	}

}
