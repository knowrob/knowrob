/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors:
 *     Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation;

import java.awt.Color;

import processing.core.PGraphics;
import edu.tum.cs.uima.Annotation;
import edu.tum.cs.vis.model.util.Mesh;
import edu.tum.cs.vis.model.util.Polygon;

/**
 * Base class for all mesh annotations.
 * 
 * @author Stefan Profanter
 * 
 */
public abstract class MeshAnnotation extends Annotation {
	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= 1222063742441634463L;

	/**
	 * Mesh which contains the referenced Polygons for which this annotation stands.
	 */
	protected Mesh				mesh				= new Mesh();

	/**
	 * indicates if this annotation should be drawn or not
	 */
	protected boolean			drawAnnotation		= true;

	/**
	 * Draw the annotation with color from <code>getAnnotationColor()</code>
	 * 
	 * @param g
	 *            Applet to draw on
	 */
	public void draw(PGraphics g) {
		if (drawAnnotation) {
			mesh.drawLines(g, getAnnotationColor());
			mesh.drawPolygons(g, getAnnotationColor());
		}
	}

	/**
	 * Returns the color for drawing this annotation. Each type of annotation should have a
	 * different color.
	 * 
	 * @return the color
	 */
	public abstract Color getAnnotationColor();

	/**
	 * @return the mesh
	 */
	public Mesh getMesh() {
		return mesh;
	}

	/**
	 * @return the drawAnnotation
	 */
	public boolean isDrawAnnotation() {
		return drawAnnotation;
	}

	/**
	 * Checks if this annotation includes the polygon <code>p</code>.
	 * 
	 * @param p
	 *            polygon to check for
	 * @return true if annotation includes polygon
	 */
	public boolean meshContainsPolygon(final Polygon p) {
		return mesh.getPolygons().contains(p);
	}

	/**
	 * @param drawAnnotation
	 *            the drawAnnotation to set
	 */
	public void setDrawAnnotation(boolean drawAnnotation) {
		this.drawAnnotation = drawAnnotation;
	}

	/**
	 * @param mesh
	 *            the mesh to set
	 */
	public void setMesh(Mesh mesh) {
		this.mesh = mesh;
	}

}
