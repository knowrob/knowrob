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

import edu.tum.cs.vis.model.uima.annotation.primitive.PlaneAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;

/**
 * @author Stefan Profanter
 * 
 */
public class PlaneAnnotationPanel extends AnnotationPanel<PlaneAnnotation> {
	/**
	 * Area of selected annotation
	 */
	private final JTextField	txtAreaTot;

	/**
	 * Area coverage of selected annotation
	 */
	private final JTextField	txtAreaCov;
	private final JTextField	txtShort;
	private final JTextField	txtLong;

	/**
	 * @param annotationType
	 * @param cas
	 */
	public PlaneAnnotationPanel(MeshCas cas) {
		super(PlaneAnnotation.class, cas);

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

		txtShort = new JTextField();
		c.gridx = 0;
		c.gridy = 2;
		c.weightx = 0.3;
		pnlInfo.add(new JLabel("Short len:", SwingConstants.CENTER), c);
		c.gridx = 1;
		c.gridy = 2;
		c.weightx = 1.0;
		pnlInfo.add(txtShort, c);

		txtLong = new JTextField();
		c.gridx = 0;
		c.gridy = 3;
		c.weightx = 0.3;
		pnlInfo.add(new JLabel("Long len:", SwingConstants.CENTER), c);
		c.gridx = 1;
		c.gridy = 3;
		c.weightx = 1.0;
		pnlInfo.add(txtLong, c);

		add(pnlInfo, BorderLayout.CENTER);

		setSelected(null);
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.view.control.AnnotationPanel#setSelected(edu.tum.cs.vis.model.uima.annotation.DrawableAnnotation)
	 */
	@Override
	public void setSelected(PlaneAnnotation annotation) {
		txtAreaTot.setEnabled(annotation != null);
		txtAreaCov.setEnabled(annotation != null);
		txtShort.setEnabled(annotation != null);
		txtLong.setEnabled(annotation != null);

		if (annotation != null) {
			txtAreaTot.setText(String.valueOf(annotation.getPrimitiveArea()));
			txtAreaCov.setText(String.valueOf(annotation.getArea() / annotation.getPrimitiveArea()
					* 100f)
					+ "%");
			txtShort.setText(String.valueOf(annotation.getShortSide().length()));
			txtLong.setText(String.valueOf(annotation.getLongSide().length()));
		}

	}

}
