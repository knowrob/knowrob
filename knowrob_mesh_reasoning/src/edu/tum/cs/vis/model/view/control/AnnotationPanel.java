/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.view.control;

import javax.swing.JPanel;

import edu.tum.cs.vis.model.uima.annotation.MeshAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;

/**
 * Panel for MeshReasoningViewControl accordion. Shows different setting controls for an annotation.
 * 
 * @author Stefan Profanter
 * 
 * @param <T>
 *            Annotation for which this panel is
 */
public abstract class AnnotationPanel<T extends MeshAnnotation> extends JPanel {

	/**
	 * Auto generated
	 */
	private static final long		serialVersionUID	= 878471632553390826L;

	/**
	 * CAS which holds list of annotations
	 */
	protected MeshCas				cas;

	/**
	 * Type of annotation for which this panel is
	 */
	Class<? extends MeshAnnotation>	annotationType;

	/**
	 * Default constructor
	 * 
	 * @param annotationType
	 *            For which annotation types the panel is
	 * @param cas
	 *            CAS which holds all annotations
	 */
	public AnnotationPanel(Class<? extends MeshAnnotation> annotationType, MeshCas cas) {
		this.annotationType = annotationType;
		this.cas = cas;
	}

	/**
	 * Shows features for the given annotation in the corresponding fields or disables them if
	 * annotation is null. Eg. used if annotation is selected by mouse.
	 * 
	 * @param annotation
	 *            Annotation to show
	 */
	public abstract void setSelected(T annotation);

	/**
	 * Each time a new annotation is added to annotations list, this function will be called
	 */
	public abstract void updateValues();
}
