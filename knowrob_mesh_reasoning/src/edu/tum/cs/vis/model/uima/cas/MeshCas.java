/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.cas;

import java.io.Serializable;
import java.util.HashMap;
import java.util.HashSet;

import org.apache.log4j.Logger;

import processing.core.PGraphics;
import edu.tum.cs.uima.Annotation;
import edu.tum.cs.uima.JCas;
import edu.tum.cs.vis.model.Model;
import edu.tum.cs.vis.model.uima.annotation.DrawableAnnotation;
import edu.tum.cs.vis.model.uima.annotation.MeshAnnotation;
import edu.tum.cs.vis.model.util.Curvature;
import edu.tum.cs.vis.model.util.DrawSettings;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * UIMA CAS for 3D meshes.
 * 
 * @author Stefan Profanter
 * 
 */
public class MeshCas extends JCas implements Serializable {

	/**
	 * Log4J Logger
	 */
	@SuppressWarnings("unused")
	private static Logger						logger				= Logger.getLogger(MeshCas.class);

	/**
	 * Auto generated
	 */
	private static final long					serialVersionUID	= 4603166505444872760L;
	/**
	 * Group which represents the mesh and its child groups
	 */
	private Model								model;

	/**
	 * Indicates if the original mesh should be drawn or not.
	 */
	private boolean								drawMesh			= true;

	/**
	 * Maps curvature property to each vertex of mesh
	 */
	private final HashMap<Vertex, Curvature>	curvatures			= new HashMap<Vertex, Curvature>();

	/**
	 * adds a new annotation to the annotations list
	 * 
	 * @param a
	 *            annotation to add
	 */
	public void addAnnotation(Annotation a) {
		synchronized (annotations) {
			annotations.add(a);
		}
	}

	/**
	 * Draw the original mesh and all the meshes of the annotations.
	 * 
	 * @param g
	 *            Applet to draw on
	 */
	public void draw(PGraphics g, DrawSettings drawSettings) {
		if (model == null)
			return;
		if (drawMesh)
			model.draw(g, drawSettings);
		synchronized (annotations) {
			for (Annotation a : annotations) {
				if (!(a instanceof DrawableAnnotation))
					continue;
				DrawableAnnotation ma = (DrawableAnnotation) a;
				ma.draw(g, drawSettings);

			}
		}
	}

	/**
	 * Searches the annotation (type equals clazz) which contains the given triangle. If no
	 * annotation found, null be returned
	 * 
	 * @param clazz
	 *            Type of the annotation to find
	 * @param p
	 *            annotation must contain this triangle
	 * @return the found annotation or null
	 */
	@SuppressWarnings({ "unchecked", "rawtypes" })
	public <T extends MeshAnnotation> T findAnnotation(Class<T> clazz, Triangle p) {
		for (Annotation a : getAnnotations()) {
			if (!(a instanceof MeshAnnotation))
				continue;
			MeshAnnotation ma = (MeshAnnotation) a;
			if ((ma.getClass() == clazz || clazz.isInstance(ma))
					&& ma.getMesh().getTriangles().contains(p))
				return (T) ma;
		}
		return null;
	}

	/**
	 * Searches all annotations which are an instance of the given class.
	 * 
	 * @param clazz
	 *            Type of the annotations to find
	 * @return the found annotations or empty
	 */
	public <T extends DrawableAnnotation> HashSet<T> findAnnotations(Class<T> clazz) {
		HashSet<T> an = new HashSet<T>();
		for (Annotation a : getAnnotations()) {
			if (!(a instanceof DrawableAnnotation))
				continue;
			DrawableAnnotation ma = (DrawableAnnotation) a;
			if (ma.getClass() == clazz || clazz.isInstance(ma)) {
				an.add((T) a);
			}
		}
		return an;
	}

	/**
	 * Get curvature property for vertex <tt>v</tt>
	 * 
	 * @param v
	 *            Vertex for which curvature is needed
	 * @return curvature properties or null if none defined
	 */
	public Curvature getCurvature(Vertex v) {
		return curvatures.get(v);
	}

	/**
	 * Get vertex curvature mapping.
	 * 
	 * @return map for vertex to curvature
	 */
	public HashMap<Vertex, Curvature> getCurvatures() {
		return curvatures;
	}

	/**
	 * Get parent model of mesh
	 * 
	 * @return the model
	 */
	public Model getModel() {
		return model;
	}

	/**
	 * Should mesh be drawn or not
	 * 
	 * @return the drawMesh
	 */
	public boolean isDrawMesh() {
		return drawMesh;
	}

	/**
	 * removes the annotation from annotations list
	 * 
	 * @param a
	 *            annotation to remove
	 */
	public void removeAnnotation(Annotation a) {
		synchronized (annotations) {
			annotations.remove(a);
		}
	}

	/**
	 * Set curvature property for given vertex. Overwrites existing one for vertex.
	 * 
	 * @param v
	 *            Vertex to which the curvature property should be assigned
	 * @param c
	 *            curvature property
	 */
	public void setCurvature(Vertex v, Curvature c) {
		curvatures.put(v, c);
	}

	/**
	 * Should mesh be drawn (be visible) or not
	 * 
	 * @param drawMesh
	 *            the drawMesh to set
	 */
	public void setDrawMesh(boolean drawMesh) {
		this.drawMesh = drawMesh;
	}

	/**
	 * Set parent model of mesh.
	 * 
	 * @param model
	 *            the model to set
	 */
	public void setModel(Model model) {
		this.model = model;
	}

}
