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

import edu.tum.cs.vis.model.uima.annotation.ContainerAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;

/**
 * Control panel for container annotation
 * 
 * @author Stefan Profanter
 * 
 */
public class ContainerAnnotationPanel extends AnnotationPanel<ContainerAnnotation> {

	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= 14768465886755070L;

	/**
	 * Area of selected annotation
	 */
	private final JTextField	txtDirection;

	/**
	 * Volume of selected annotation
	 */
	private final JTextField	txtVolume;

	/**
	 * Creates new container annotation panel
	 * 
	 * @param cas
	 *            main mesh cas
	 */
	public ContainerAnnotationPanel(MeshCas cas) {
		super(ContainerAnnotation.class, cas);

		setLayout(new BorderLayout());

		GridBagConstraints c = new GridBagConstraints();
		c.fill = GridBagConstraints.HORIZONTAL;

		add(getColorPanel(), BorderLayout.NORTH);

		JPanel pnlInfo = new JPanel(new GridBagLayout());

		txtDirection = new JTextField();
		c.gridx = 0;
		c.gridy = 0;
		c.weightx = 0.3;
		pnlInfo.add(new JLabel("Direction", SwingConstants.CENTER), c);
		c.gridx = 1;
		c.gridy = 0;
		c.weightx = 1.0;
		pnlInfo.add(txtDirection, c);

		txtVolume = new JTextField();
		c.gridx = 0;
		c.gridy = 1;
		c.weightx = 0.3;
		pnlInfo.add(new JLabel("Volume", SwingConstants.CENTER), c);
		c.gridx = 1;
		c.gridy = 1;
		c.weightx = 1.0;
		pnlInfo.add(txtVolume, c);

		add(pnlInfo, BorderLayout.CENTER);

		setSelected(null);
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.view.control.AnnotationPanel#setSelected(edu.tum.cs.vis.model.uima.annotation.DrawableAnnotation)
	 */
	@Override
	public void setSelected(ContainerAnnotation annotation) {
		txtDirection.setEnabled(annotation != null);
		txtVolume.setEnabled(annotation != null);

		if (annotation != null) {
			txtDirection.setText(String.valueOf(annotation.getDirectionUnscaled()));
			txtVolume.setText(String.valueOf(annotation.getVolumeUnscaled()));
		}

	}

}
