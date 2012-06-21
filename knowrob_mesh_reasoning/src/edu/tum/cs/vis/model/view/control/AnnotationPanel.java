/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.view.control;

import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.ButtonGroup;
import javax.swing.JPanel;
import javax.swing.JRadioButton;

import edu.tum.cs.uima.Annotation;
import edu.tum.cs.vis.model.uima.annotation.DrawableAnnotation;
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
public abstract class AnnotationPanel<T extends DrawableAnnotation> extends JPanel implements
		ActionListener {

	/**
	 * Auto generated
	 */
	private static final long			serialVersionUID	= 878471632553390826L;

	/**
	 * CAS which holds list of annotations
	 */
	protected MeshCas					cas;

	/**
	 * Type of annotation for which this panel is
	 */
	Class<? extends DrawableAnnotation>	annotationType;

	/**
	 * Default constructor
	 * 
	 * @param annotationType
	 *            For which annotation types the panel is
	 * @param cas
	 *            CAS which holds all annotations
	 */
	public AnnotationPanel(Class<? extends DrawableAnnotation> annotationType, MeshCas cas) {
		this.annotationType = annotationType;
		this.cas = cas;
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		boolean useRand = false;
		System.out.println("action");
		if (e.getActionCommand() == "default_color")
			useRand = false;
		else if (e.getActionCommand() == "random_color")
			useRand = true;
		else
			return;

		synchronized (cas.getAnnotations()) {
			for (Annotation a : cas.getAnnotations()) {
				if (a.getClass() != annotationType) {
					continue;
				}
				MeshAnnotation ma = (MeshAnnotation) a;
				ma.setUseRandomColor(useRand);
			}
		}
	}

	/**
	 * Creates the panel for "Default color" and "Random color" radio button and connects the action
	 * listener. You have to add this panel to your container and nothing else.
	 * 
	 * @return the initialized panel.
	 */
	protected JPanel getColorPanel() {
		JPanel pnl = new JPanel(new GridBagLayout());
		JRadioButton defButton = new JRadioButton("Default color");
		defButton.setActionCommand("default_color");
		defButton.setSelected(true);

		JRadioButton randColor = new JRadioButton("Random color");
		randColor.setActionCommand("random_color");

		// Group the radio buttons.
		ButtonGroup group = new ButtonGroup();
		group.add(defButton);
		group.add(randColor);

		// Register a listener for the radio buttons.
		defButton.addActionListener(this);
		randColor.addActionListener(this);

		GridBagConstraints c = new GridBagConstraints();
		c.fill = GridBagConstraints.HORIZONTAL;
		c.weightx = 1.0;

		c.gridx = 0;
		c.gridy = 0;
		pnl.add(defButton, c);
		c.gridx = 1;
		c.gridy = 0;
		pnl.add(randColor, c);

		return pnl;
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
