/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.cas;

import java.io.Serializable;
import java.util.HashSet;

import processing.core.PGraphics;
import edu.tum.cs.uima.Annotation;
import edu.tum.cs.uima.JCas;
import edu.tum.cs.vis.model.Model;
import edu.tum.cs.vis.model.uima.annotation.DrawableAnnotation;
import edu.tum.cs.vis.model.uima.annotation.MeshAnnotation;
import edu.tum.cs.vis.model.util.Triangle;

/**
 * UIMA CAS for 3D meshes.
 * 
 * @author Stefan Profanter
 * 
 */
public class MeshCas extends JCas implements Serializable {

	/**
	 * Auto generated
	 */
	private static final long	serialVersionUID	= 4603166505444872760L;
	/**
	 * Group which represents the mesh and its child groups
	 */
	private Model				model;

	/**
	 * Indicates if the original mesh should be drawn or not.
	 */
	private boolean				drawMesh			= true;

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
	public void draw(PGraphics g) {
		if (drawMesh)
			model.draw(g, null);
		synchronized (annotations) {
			for (Annotation a : annotations) {
				if (!(a instanceof DrawableAnnotation))
					continue;
				DrawableAnnotation ma = (DrawableAnnotation) a;
				ma.draw(g);

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
	public MeshAnnotation findAnnotation(Class<? extends MeshAnnotation> clazz, Triangle p) {
		for (Annotation a : getAnnotations()) {
			if (!(a instanceof MeshAnnotation))
				continue;
			MeshAnnotation ma = (MeshAnnotation) a;
			if ((ma.getClass() == clazz || clazz.isInstance(ma))
					&& ma.getMesh().getTriangles().contains(p))
				return ma;
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
	public HashSet<MeshAnnotation> findAnnotations(Class<? extends MeshAnnotation> clazz) {
		HashSet<MeshAnnotation> an = new HashSet<MeshAnnotation>();
		for (Annotation a : getAnnotations()) {
			if (!(a instanceof MeshAnnotation))
				continue;
			an.add((MeshAnnotation) a);
		}
		return an;
	}

	/**
	 * @return the model
	 */
	public Model getModel() {
		return model;
	}

	/**
	 * @return the drawMesh
	 */
	public boolean isDrawMesh() {
		return drawMesh;
	}

	/**
	 * @param drawMesh
	 *            the drawMesh to set
	 */
	public void setDrawMesh(boolean drawMesh) {
		this.drawMesh = drawMesh;
	}

	/**
	 * @param model
	 *            the model to set
	 */
	public void setModel(Model model) {
		this.model = model;
	}

}
