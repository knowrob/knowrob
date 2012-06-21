package edu.tum.cs.vis.model.uima.annotation;

import java.awt.Color;

import processing.core.PGraphics;
import edu.tum.cs.uima.Annotation;
import edu.tum.cs.vis.model.util.Mesh;

/**
 * Base class for all mesh annotations.
 * 
 * @author Stefan Profanter
 * 
 */
public abstract class MeshAnnotation extends Annotation {
	/**
	 * Mesh which contains the referenced Polygons for which this annotation stands.
	 */
	protected Mesh		mesh			= new Mesh();

	protected boolean	drawAnnotation	= true;

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
