/*******************************************************************************
 * Copyright (c) 2013 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2013
 ******************************************************************************/
package edu.tum.cs.vis.model.view;

import java.awt.BorderLayout;
import java.awt.Frame;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingConstants;

/**
 * @author Stefan Profanter
 * 
 */
public class ControlInfoDialog extends JDialog implements ActionListener {

	private final JButton			closeBtn;
	private JPanel					myPanel		= null;

	private final static String[][]	shortcuts	= new String[][] {
			{ "left click", "Camera rotation" },
			{ "right click", "Camera move" },
			{ "mouse wheel", "Camera zoom" },
			{ ",", "Zoom in" },
			{ ".", "Zoom out" },
			{ "+", "Increase scale" },
			{ "-", "Decrease scale" },
			{ "1", "Draw mode: Fill" },
			{ "2", "Draw mode: Lines" },
			{ "3", "Draw mode: Points" },
			{ "q", "Increase line width" },
			{ "w", "Decrease line width" },
			{ "<html>Shift+Click on<br />Annotation checkbox</html>",
			"<html>Toggle all<br />other annotations</html>" } };

	public ControlInfoDialog(Frame owner) {
		super(owner, true);
		setTitle("Control info");
		myPanel = new JPanel(new BorderLayout());
		getContentPane().add(myPanel);

		JPanel pnlLeft = new JPanel(new GridBagLayout());
		JPanel pnlCenter = new JPanel(new GridBagLayout());
		myPanel.add(pnlLeft, BorderLayout.WEST);
		myPanel.add(pnlCenter, BorderLayout.CENTER);

		GridBagConstraints c = new GridBagConstraints();
		c.fill = GridBagConstraints.HORIZONTAL;
		c.insets = new Insets(10, 10, 10, 10);

		for (int i = 0; i < shortcuts.length; i++) {
			c.gridx = 0;
			c.gridy = i;
			pnlLeft.add(new JLabel(shortcuts[i][0], SwingConstants.RIGHT), c);
			c.gridx = 0;
			c.gridy = i;
			pnlCenter.add(new JLabel(shortcuts[i][1]), c);
		}

		closeBtn = new JButton("close");
		closeBtn.addActionListener(this);
		myPanel.add(closeBtn, BorderLayout.SOUTH);
		pack();
		setLocationRelativeTo(owner);
		setVisible(true);

	}

	@Override
	public void actionPerformed(ActionEvent e) {
		if (closeBtn == e.getSource()) {
			setVisible(false);
		}
	}
}
