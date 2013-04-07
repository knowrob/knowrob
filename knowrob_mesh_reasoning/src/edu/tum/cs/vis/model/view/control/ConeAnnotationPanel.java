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
import java.text.DecimalFormat;

import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.SwingConstants;

import edu.tum.cs.vis.model.uima.annotation.primitive.ConeAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.HandleComparator;

/**
 * Control panel for cone annotation.
 * 
 * @author Stefan Profanter
 * 
 */
public class ConeAnnotationPanel extends AnnotationPanel<ConeAnnotation> {

	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= -4899042762344011422L;

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
	 * normal vector of selected annotation
	 */
	private final JTextField	txtDirectionVector;

	/**
	 * Fitting error
	 */
	private final JTextField	txtFitError;

	/**
	 * textfield for handle weight
	 */
	private final JTextField	txtHandleWeight;

	/**
	 * height of cone
	 */
	private final JTextField	txtHeight;

	/**
	 * Creates new cone annotation panel
	 * 
	 * @param cas
	 *            main mesh cas
	 */
	public ConeAnnotationPanel(MeshCas cas) {
		super(ConeAnnotation.class, cas);

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

		txtDirectionVector = new JTextField();
		c.gridx = 0;
		c.gridy = 3;
		c.weightx = 0.3;
		pnlInfo.add(new JLabel("Direction", SwingConstants.CENTER), c);
		c.gridx = 1;
		c.gridy = 3;
		c.weightx = 1.0;
		pnlInfo.add(txtDirectionVector, c);

		txtHeight = new JTextField();
		c.gridx = 0;
		c.gridy = 4;
		c.weightx = 0.3;
		pnlInfo.add(new JLabel("Height", SwingConstants.CENTER), c);
		c.gridx = 1;
		c.gridy = 4;
		c.weightx = 1.0;
		pnlInfo.add(txtHeight, c);

		txtFitError = new JTextField();
		c.gridx = 0;
		c.gridy = 5;
		c.weightx = 0.3;
		pnlInfo.add(new JLabel("FitError", SwingConstants.CENTER), c);
		c.gridx = 1;
		c.gridy = 5;
		c.weightx = 1.0;
		pnlInfo.add(txtFitError, c);

		txtHandleWeight = new JTextField();
		c.gridx = 0;
		c.gridy = 6;
		c.weightx = 0.3;
		pnlInfo.add(new JLabel("HandleWeight", SwingConstants.CENTER), c);
		c.gridx = 1;
		c.gridy = 6;
		c.weightx = 1.0;
		pnlInfo.add(txtHandleWeight, c);

		add(pnlInfo, BorderLayout.CENTER);

		setSelected(null);

	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.view.control.AnnotationPanel#setSelected(edu.tum.cs.vis.model.uima.annotation.DrawableAnnotation)
	 */
	@Override
	public void setSelected(ConeAnnotation annotation) {
		txtAreaTot.setEnabled(annotation != null);
		txtAreaCov.setEnabled(annotation != null);
		txtRadius.setEnabled(annotation != null);
		txtDirectionVector.setEnabled(annotation != null);
		txtHeight.setEnabled(annotation != null);
		txtFitError.setEnabled(annotation != null);
		txtHandleWeight.setEnabled(annotation != null);

		if (annotation != null) {
			txtAreaTot.setText(String.valueOf(annotation.getPrimitiveArea()));
			txtAreaCov.setText(String.valueOf(annotation.getAreaCoverage() * 100f) + "%");
			txtRadius.setText(String.valueOf(annotation.getRadiusLarge()) + " -> "
					+ String.valueOf(annotation.getRadiusSmall()));
			txtDirectionVector.setText(annotation.getDirection().toString());
			txtHeight.setText(String.valueOf(annotation.getDirection().length() * 2));
			txtFitError.setText(String.valueOf(annotation.getCone().getFitError()));

			DecimalFormat scientForm = new DecimalFormat("0.####E0");
			DecimalFormat normForm = new DecimalFormat("0.######");
			double weight = HandleComparator.getHandleWeight(annotation, annotation.getModel(),
					HandleComparator.DEFAULT_RADIUS_MIN, HandleComparator.DEFAULT_RADIUS_MAX,
					HandleComparator.DEFAULT_LENGTH_MIN, HandleComparator.DEFAULT_LENGTH_MAX);
			String str = weight < 0.00001 ? scientForm.format(weight) : normForm.format(weight);
			txtHandleWeight.setText(String.valueOf(annotation.isConcave() ? 0 : str));
		}

	}
}
