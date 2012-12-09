/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation;

import java.util.HashMap;

import edu.tum.cs.vis.model.Model;
import edu.tum.cs.vis.model.uima.annotation.primitive.ConeAnnotation;
import edu.tum.cs.vis.model.util.Curvature;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * Annotation for a complex handle.
 * 
 * A complex handle is a convex part in the model.
 * 
 * 
 * @author Stefan Profanter
 * 
 */
public class ComplexHandleAnnotation extends ConeAnnotation {

	/**
	 * generated
	 */
	private static final long	serialVersionUID	= 1983829446921229660L;

	/**
	 * Create new complex handle annotation.
	 * 
	 * @param curvatures
	 *            Map of curvatures for vertices
	 * @param model
	 *            parent model
	 */
	public ComplexHandleAnnotation(HashMap<Vertex, Curvature> curvatures, Model model) {
		super(curvatures, model, false);
	}

	/**
	 * @param an
	 * @param t
	 */
	public void merge(ComplexHandleAnnotation an) {
		if (an != null) {
			synchronized (getMesh().getTriangles()) {
				getMesh().getTriangles().addAll(an.getMesh().getTriangles());
			}
		}

	}

}
