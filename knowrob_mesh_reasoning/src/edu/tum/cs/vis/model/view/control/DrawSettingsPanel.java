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
 * Control panel for draw settings
 * 
 * @author Stefan Profanter
 * 
 */
public class DrawSettingsPanel extends JPanel implements ActionListener {

	/**
	 * auto generated
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

	/**
	 * use white background
	 */
	private final JCheckBox			cbxWhiteBackground;
	/**
	 * draw vertex normals
	 */
	private final JCheckBox			cbxDrawVertexNormals;
	/**
	 * draw vertex curvature
	 */
	private final JCheckBox			cbxDrawVertexCurvature;
	/**
	 * color model by curvature
	 */
	private final JCheckBox			cbxDrawCurvatureColor;
	/**
	 * draw voronoi area
	 */
	private final JCheckBox			cbxDrawVoronoiArea;
	/**
	 * button to set rotation of camera manually
	 */
	private final JButton			btnSetRotation;
	/**
	 * select only nearest triangle or all triangles intersecting mouse ray
	 */
	private final JCheckBox			cbxSelectNearestOnly;
	/**
	 * draw bounding box for each group
	 */
	private final JCheckBox			cbxDrawBoundingBox;

	/**
	 * Creates a new draw settings panel
	 * 
	 * @param cas
	 *            main mesh cas
	 * @param view
	 *            parent mesh reasoning view
	 */
	public DrawSettingsPanel(MeshCas cas, MeshReasoningView view) {
		this.view = view;
		this.cas = cas;
		// setPreferredSize(new Dimension(300, 300));

		GridLayout grid = new GridLayout(0, 2);
		setLayout(grid);

		cbxWhiteBackground = new JCheckBox("White Background");
		cbxWhiteBackground.addActionListener(this);
		cbxWhiteBackground.setSelected(view.isBackgroundWhite());
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

		cbxDrawVoronoiArea = new JCheckBox("Voronoi Area");
		cbxDrawVoronoiArea.addActionListener(this);
		cbxDrawVoronoiArea.setSelected(false);
		this.add(cbxDrawVoronoiArea);

		btnSetRotation = new JButton("Set View");
		btnSetRotation.addActionListener(this);
		this.add(btnSetRotation);

		cbxSelectNearestOnly = new JCheckBox("Select nearest");
		cbxSelectNearestOnly.addActionListener(this);
		cbxSelectNearestOnly.setSelected(view.isSelectNearestOnly());
		this.add(cbxSelectNearestOnly);

		cbxDrawBoundingBox = new JCheckBox("Bounding box");
		cbxDrawBoundingBox.addActionListener(this);
		cbxDrawBoundingBox.setSelected(view.isDrawBoundingBox());
		this.add(cbxDrawBoundingBox);

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
		else if (e.getSource() == cbxDrawVoronoiArea)
			view.setDrawVoronoiArea(cbxDrawVoronoiArea.isSelected());
		else if (e.getSource() == cbxDrawBoundingBox)
			view.setDrawBoundingBox(cbxDrawBoundingBox.isSelected());
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
		} else if (e.getSource() == cbxSelectNearestOnly)
			view.setSelectNearestOnly(cbxSelectNearestOnly.isSelected());
	}
}
