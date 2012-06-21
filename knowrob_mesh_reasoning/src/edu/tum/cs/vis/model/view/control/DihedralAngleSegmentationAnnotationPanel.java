/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.view.control;

import java.awt.BorderLayout;
import java.awt.GridBagLayout;

import javax.swing.JPanel;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import edu.tum.cs.vis.model.uima.annotation.DihedralAngleSegmentationAnnotation;
import edu.tum.cs.vis.model.uima.annotation.FlatSurfaceAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;

/**
 * @author Stefan Profanter
 * 
 */
public class DihedralAngleSegmentationAnnotationPanel extends
		AnnotationPanel<DihedralAngleSegmentationAnnotation> implements ChangeListener {

	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= 267498563413680156L;

	/**
	 * Default constructor. Sets panel content.
	 * 
	 * @param cas
	 *            Main cas
	 */
	public DihedralAngleSegmentationAnnotationPanel(MeshCas cas) {
		super(DihedralAngleSegmentationAnnotation.class, cas);
		setLayout(new BorderLayout());

		JPanel pnlSettings = new JPanel(new GridBagLayout());

		add(getColorPanel(), BorderLayout.NORTH);

		add(pnlSettings, BorderLayout.CENTER);

		setSelected(null);

		updateValues();
	}

	@Override
	public void setSelected(DihedralAngleSegmentationAnnotation annotation) {

	}

	@Override
	public void stateChanged(ChangeEvent arg0) {

	}

	@Override
	public void updateValues() {

	}

}
