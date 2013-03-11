/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.view.control;

import java.awt.BorderLayout;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;

import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.SwingConstants;

import edu.tum.cs.vis.model.uima.annotation.primitive.SphereAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;

/**
 * Control panel for sphere annotation.
 * 
 * @author Stefan Profanter
 * 
 */
public class SphereAnnotationPanel extends AnnotationPanel<SphereAnnotation> {

	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= -7784059552068531420L;

	/**
	 * Area of selected annotation
	 */
	private final JTextField	txtAreaTot;

	/**
	 * Area coverage of selected annotation
	 */
	private final JTextField	txtAreaCov;
	/**
	 * dimension of selected annotation
	 */
	private final JTextField	txtRadius;

	/**
	 * Fitting error
	 */
	private final JTextField	txtFitError;

	/**
	 * Creates new sphere control panel
	 * 
	 * @param cas
	 *            main mesh CAS
	 */
	public SphereAnnotationPanel(MeshCas cas) {
		super(SphereAnnotation.class, cas);

		setLayout(new BorderLayout());

		GridBagConstraints c = new GridBagConstraints();
		c.fill = GridBagConstraints.HORIZONTAL;

		add(getColorPanel(), BorderLayout.NORTH);

		JPanel pnlInfo = new JPanel(new GridBagLayout());

		txtAreaTot = new JTextField();
		c.gridx = 0;
		c.gridy = 0;
		c.weightx = 0.3;
		pnlInfo.add(new JLabel("Area Total", SwingConstants.CENTER), c);
		c.gridx = 1;
		c.gridy = 0;
		c.weightx = 1.0;
		pnlInfo.add(txtAreaTot, c);

		txtAreaCov = new JTextField();
		c.gridx = 0;
		c.gridy = 1;
		c.weightx = 0.3;
		pnlInfo.add(new JLabel("Area coverage", SwingConstants.CENTER), c);
		c.gridx = 1;
		c.gridy = 1;
		c.weightx = 1.0;
		pnlInfo.add(txtAreaCov, c);

		txtRadius = new JTextField();
		c.gridx = 0;
		c.gridy = 2;
		c.weightx = 0.3;
		pnlInfo.add(new JLabel("Radius", SwingConstants.CENTER), c);
		c.gridx = 1;
		c.gridy = 2;
		c.weightx = 1.0;
		pnlInfo.add(txtRadius, c);

		txtFitError = new JTextField();
		c.gridx = 0;
		c.gridy = 3;
		c.weightx = 0.3;
		pnlInfo.add(new JLabel("FitError", SwingConstants.CENTER), c);
		c.gridx = 1;
		c.gridy = 3;
		c.weightx = 1.0;
		pnlInfo.add(txtFitError, c);

		add(pnlInfo, BorderLayout.CENTER);

		setSelected(null);
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.view.control.AnnotationPanel#setSelected(edu.tum.cs.vis.model.uima.annotation.DrawableAnnotation)
	 */
	@Override
	public void setSelected(SphereAnnotation annotation) {
		txtAreaTot.setEnabled(annotation != null);
		txtAreaCov.setEnabled(annotation != null);
		txtRadius.setEnabled(annotation != null);
		txtFitError.setEnabled(annotation != null);

		if (annotation != null) {
			txtAreaTot.setText(String.valueOf(annotation.getPrimitiveAreaUnscaled()));
			txtAreaCov.setText(String.valueOf(annotation.getAreaCoverage() * 100f) + "%");
			txtRadius.setText(String.valueOf(annotation.getRadiusUnscaled()));
			txtFitError.setText(String.valueOf(annotation.getSphere().getFitError()));

		}

	}

}
