/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.view.control;

import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JOptionPane;
import javax.swing.JPanel;

import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.view.MeshReasoningView;

/**
 * @author Stefan Profanter
 * 
 */
public class DrawSettingsPanel extends JPanel implements ActionListener {

	/**
	 * 
	 */
	private static final long		serialVersionUID	= -1936392151575823886L;

	/**
	 * cas for which this control is
	 */
	@SuppressWarnings("unused")
	private final MeshCas			cas;

	/**
	 * The view for which this control is
	 */
	private final MeshReasoningView	view;

	private final JCheckBox			cbxWhiteBackground;
	private final JCheckBox			cbxDrawVertexNormals;
	private final JCheckBox			cbxDrawVertexCurvature;
	private final JCheckBox			cbxDrawCurvatureColor;
	private final JButton			btnSetRotation;

	public DrawSettingsPanel(MeshCas cas, MeshReasoningView view) {
		this.view = view;
		this.cas = cas;
		// setPreferredSize(new Dimension(300, 300));

		GridLayout grid = new GridLayout(0, 2);
		setLayout(grid);

		cbxWhiteBackground = new JCheckBox("White Background");
		cbxWhiteBackground.addActionListener(this);
		cbxWhiteBackground.setSelected(false);
		this.add(cbxWhiteBackground);

		cbxDrawVertexNormals = new JCheckBox("Vertex normals");
		cbxDrawVertexNormals.addActionListener(this);
		cbxDrawVertexNormals.setSelected(false);
		this.add(cbxDrawVertexNormals);

		cbxDrawVertexCurvature = new JCheckBox("Vertex curvature");
		cbxDrawVertexCurvature.addActionListener(this);
		cbxDrawVertexCurvature.setSelected(false);
		this.add(cbxDrawVertexCurvature);

		cbxDrawCurvatureColor = new JCheckBox("Color by curvature");
		cbxDrawCurvatureColor.addActionListener(this);
		cbxDrawCurvatureColor.setSelected(false);
		this.add(cbxDrawCurvatureColor);

		btnSetRotation = new JButton("Set View");
		btnSetRotation.addActionListener(this);
		this.add(btnSetRotation);

	}

	@Override
	public void actionPerformed(ActionEvent e) {
		if (e.getSource() == cbxWhiteBackground)
			view.setBackgroundWhite(cbxWhiteBackground.isSelected());
		else if (e.getSource() == cbxDrawVertexNormals)
			view.setDrawVertexNormals(cbxDrawVertexNormals.isSelected());
		else if (e.getSource() == cbxDrawVertexCurvature)
			view.setDrawVertexCurvature(cbxDrawVertexCurvature.isSelected());
		else if (e.getSource() == cbxDrawCurvatureColor)
			view.setDrawCurvatureColor(cbxDrawCurvatureColor.isSelected());
		else if (e.getSource() == btnSetRotation) {
			String current = Math.round(view.getRotation()[0] * 180f / Math.PI) + ","
					+ Math.round(view.getRotation()[1] * 180f / Math.PI) + ","
					+ Math.round(view.getRotation()[2] * 180f / Math.PI);

			String s = (String) JOptionPane.showInputDialog(this,
					"Enter the view parameters (in degree) in the format 'pitch,yaw,roll'",
					"View parameters", JOptionPane.PLAIN_MESSAGE, null, null, current);

			// If a string was returned, say so.
			if ((s != null) && (s.length() > 0)) {
				String parts[] = s.split(",");
				if (parts.length != 3) {
					JOptionPane.showMessageDialog(this, "Invalid format.", "Input error",
							JOptionPane.ERROR_MESSAGE);
					return;
				}

				try {
					int pitch = Integer.parseInt(parts[0]);
					int yaw = Integer.parseInt(parts[1]);
					int roll = Integer.parseInt(parts[2]);
					view.setManualRotation((float) (pitch * Math.PI / 180f),
							(float) (yaw * Math.PI / 180f), (float) (roll * Math.PI / 180f));
				} catch (NumberFormatException ex) {
					JOptionPane.showMessageDialog(this, "Invalid format.", "Input error",
							JOptionPane.ERROR_MESSAGE);
					return;
				}
			}
		}
	}
}
