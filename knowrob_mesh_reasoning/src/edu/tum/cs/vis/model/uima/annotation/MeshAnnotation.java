package edu.tum.cs.vis.model.uima.annotation;

import java.awt.Color;

import processing.core.PApplet;
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
	protected Mesh	mesh	= new Mesh();

	/**
	 * Draw the annotation with color from <code>getAnnotationColor()</code>
	 * 
	 * @param applet
	 *            Applet to draw on
	 */
	public void draw(PApplet applet) {
		mesh.drawLines(applet, getAnnotationColor());
		mesh.drawPolygons(applet, getAnnotationColor());
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
	 * @param mesh
	 *            the mesh to set
	 */
	public void setMesh(Mesh mesh) {
		this.mesh = mesh;
	}

}
