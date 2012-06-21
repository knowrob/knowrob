/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.view.control;

import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import edu.tum.cs.vis.model.uima.annotation.primitive.ConeAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;

/**
 * @author Stefan Profanter
 * 
 */
public class ConeAnnotationPanel extends AnnotationPanel<ConeAnnotation> implements ChangeListener {
	/**
	 * @param annotationType
	 * @param cas
	 */
	public ConeAnnotationPanel(MeshCas cas) {
		super(ConeAnnotation.class, cas);
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.view.control.AnnotationPanel#setSelected(edu.tum.cs.vis.model.uima.annotation.DrawableAnnotation)
	 */
	@Override
	public void setSelected(ConeAnnotation annotation) {
		// TODO Auto-generated method stub

	}

	/* (non-Javadoc)
	 * @see javax.swing.event.ChangeListener#stateChanged(javax.swing.event.ChangeEvent)
	 */
	@Override
	public void stateChanged(ChangeEvent arg0) {
		// TODO Auto-generated method stub

	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.view.control.AnnotationPanel#updateValues()
	 */
	@Override
	public void updateValues() {
		// TODO Auto-generated method stub

	}

}
