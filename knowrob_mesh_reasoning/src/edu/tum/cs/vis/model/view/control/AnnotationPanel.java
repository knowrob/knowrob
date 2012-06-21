/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.view.control;

import java.util.ArrayList;

import javax.swing.JPanel;

import edu.tum.cs.vis.model.uima.annotation.MeshAnnotation;

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
	private static final long	serialVersionUID	= 878471632553390826L;
	/**
	 * List of annotations connected with this panel
	 */
	protected ArrayList<T>		annotations;

	/**
	 * @return the annotations
	 */
	public ArrayList<T> getAnnotations() {
		return annotations;
	}

	/**
	 * @param annotations
	 *            the annotations to set
	 */
	public void setAnnotations(ArrayList<T> annotations) {
		this.annotations = annotations;
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
