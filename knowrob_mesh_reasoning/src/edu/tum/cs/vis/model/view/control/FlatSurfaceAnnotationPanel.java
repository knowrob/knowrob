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

import javax.swing.BorderFactory;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.JTextField;
import javax.swing.SwingConstants;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import edu.tum.cs.uima.Annotation;
import edu.tum.cs.vis.model.uima.annotation.FlatSurfaceAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;

/**
 * Control panel for FlatSurfaceAnnotation. Contains slider for minArea and maxArea.
 * 
 * @author Stefan Profanter
 * 
 */
public class FlatSurfaceAnnotationPanel extends AnnotationPanel<FlatSurfaceAnnotation> implements
		ChangeListener {

	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= 267498563413680156L;
	/**
	 * Slider for minArea
	 */
	private final JSlider		minArea;
	/**
	 * Slider for maxArea
	 */
	private final JSlider		maxArea;

	/**
	 * Slider can't represent float. So we use a factor and convert the values to int.
	 */
	private final static int	FACTOR				= 100;

	/**
	 * Panel which contains text fields filled when specific annotation is selected
	 */
	private final JPanel		pnlInfo;

	/**
	 * Area of selected annotation
	 */
	private final JTextField	txtArea;
	/**
	 * dimension of selected annotation
	 */
	private final JTextField	txtDimension;
	/**
	 * normal vector of selected annotation
	 */
	private final JTextField	txtNormalVector;

	/**
	 * Default constructor. Sets panel content.
	 * 
	 * @param cas
	 *            Main cas
	 */
	public FlatSurfaceAnnotationPanel(MeshCas cas) {
		super(FlatSurfaceAnnotation.class, cas);
		setLayout(new BorderLayout());

		JPanel pnlSettings = new JPanel(new GridBagLayout());

		GridBagConstraints c = new GridBagConstraints();
		c.fill = GridBagConstraints.HORIZONTAL;
		c.weightx = 1.0;
		JLabel sliderLabel = new JLabel("Min. area", SwingConstants.CENTER);
		minArea = new JSlider(SwingConstants.HORIZONTAL, 0, 1, 0);

		minArea.addChangeListener(this);

		// Turn on labels at major tick marks.

		c.gridx = 0;
		c.gridy = 0;
		pnlSettings.add(sliderLabel, c);
		c.gridx = 1;
		c.gridy = 0;
		pnlSettings.add(minArea, c);

		JLabel sliderLabelMax = new JLabel("Max. area", SwingConstants.CENTER);
		maxArea = new JSlider(SwingConstants.HORIZONTAL, 0, 1, 1);

		maxArea.addChangeListener(this);

		// Turn on labels at major tick marks.

		c.gridx = 0;
		c.gridy = 1;
		pnlSettings.add(sliderLabelMax, c);
		c.gridx = 1;
		c.gridy = 1;
		pnlSettings.add(maxArea, c);

		add(pnlSettings, BorderLayout.CENTER);

		pnlInfo = new JPanel(new GridBagLayout());

		JLabel areaLabel = new JLabel("Area", SwingConstants.CENTER);
		txtArea = new JTextField();
		c.gridx = 0;
		c.gridy = 0;
		c.weightx = 0.5;
		pnlInfo.add(areaLabel, c);
		c.gridx = 1;
		c.gridy = 0;
		c.weightx = 1.0;
		pnlInfo.add(txtArea, c);

		JLabel dimLabel = new JLabel("Dimension", SwingConstants.CENTER);
		txtDimension = new JTextField();
		c.gridx = 0;
		c.gridy = 1;
		c.weightx = 0.5;
		pnlInfo.add(dimLabel, c);
		c.gridx = 1;
		c.gridy = 1;
		c.weightx = 1.0;
		pnlInfo.add(txtDimension, c);

		JLabel normalLabel = new JLabel("Normal", SwingConstants.CENTER);
		txtNormalVector = new JTextField();
		c.gridx = 0;
		c.gridy = 2;
		c.weightx = 0.5;
		pnlInfo.add(normalLabel, c);
		c.gridx = 1;
		c.gridy = 2;
		c.weightx = 1.0;
		pnlInfo.add(txtNormalVector, c);

		pnlInfo.setBorder(BorderFactory.createTitledBorder("Selected info"));

		add(pnlInfo, BorderLayout.SOUTH);

		setSelected(null);

		updateValues();
	}

	@Override
	public void setSelected(FlatSurfaceAnnotation annotation) {
		if (annotation == null) {
			pnlInfo.setEnabled(false);
			txtArea.setEnabled(false);
			txtDimension.setEnabled(false);
			txtNormalVector.setEnabled(false);
		} else {
			pnlInfo.setEnabled(true);
			txtArea.setText(annotation.getArea().toString());
			txtDimension.setText(annotation.getDimension().toString());
			txtNormalVector.setText(annotation.getNormalVector().toString());
			txtArea.setEnabled(true);
			txtDimension.setEnabled(true);
			txtNormalVector.setEnabled(true);
		}

	}

	@Override
	public void stateChanged(ChangeEvent arg0) {
		synchronized (cas.getAnnotations()) {
			for (Annotation a : cas.getAnnotations()) {
				if (a.getClass() != annotationType)
					continue;
				FlatSurfaceAnnotation fa = (FlatSurfaceAnnotation) a;
				fa.setDrawAnnotation(fa.getArea().getSquareMM() * FACTOR <= maxArea.getValue()
						&& fa.getArea().getSquareMM() * FACTOR >= minArea.getValue());
			}
		}

	}

	@Override
	public void updateValues() {

		double min = Double.MAX_VALUE;
		double max = Double.MIN_VALUE;

		synchronized (cas.getAnnotations()) {
			for (Annotation a : cas.getAnnotations()) {
				if (a.getClass() != annotationType)
					continue;
				FlatSurfaceAnnotation fa = (FlatSurfaceAnnotation) a;
				min = Math.min(fa.getArea().getSquareMM(), min);
				max = Math.max(fa.getArea().getSquareMM(), max);
			}
		}

		minArea.setMinimum((int) min * FACTOR);
		minArea.setMaximum((int) (max + 1) * FACTOR);
		maxArea.setMinimum((int) min * FACTOR);
		maxArea.setMaximum((int) (max + 1) * FACTOR);
		maxArea.setValue((int) (max + 1) * FACTOR);
	}
}
