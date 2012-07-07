/**
 * 
 */
package edu.tum.cs.vis.model.view;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.DefaultListModel;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JList;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.ListSelectionModel;
import javax.swing.ScrollPaneConstants;

import edu.tum.cs.vis.model.uima.analyser.MeshAnalyser;
import edu.tum.cs.vis.model.uima.annotation.MeshAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.view.control.DrawSettingsPanel;

/**
 * Control panel for MeshReasoningView. Contains different status information and controls which
 * impact the visible parts of the model.
 * 
 * @author Stefan Profanter
 * 
 */
public final class MeshReasoningViewControl extends JPanel implements ActionListener {

	/**
	 * auto generated
	 */
	private static final long				serialVersionUID		= -2013371551022209950L;

	/**
	 * list of all analysers
	 */
	private final ArrayList<MeshAnalyser>	analyser;
	/**
	 * The accordion component which contains controls for each different annotation type
	 */
	private final MeshCasAccordion			accordion;

	/**
	 * List which shows the current progress of all analysers
	 */
	private final JList						analyserList;
	/**
	 * List model for analyserList
	 */
	private final DefaultListModel			analyserListModel;

	private final DrawSettingsPanel			drawSettingsPanel;

	/**
	 * check box to enable or disable drawing of main mesh (base data)
	 */
	private final JCheckBox					cbxShowMesh;

	/**
	 * Button to save current view into an image file
	 */
	private final JButton					btnSave;

	/**
	 * cas for which this control is
	 */
	private final MeshCas					cas;

	/**
	 * The view for which this control is
	 */
	private final MeshReasoningView			view;

	private String							defaultImageFilename	= null;

	/**
	 * Default constructor
	 * 
	 * @param cas
	 *            cas for which the control is
	 * @param analyser
	 *            list of analysers used
	 * @param view
	 *            the parent MeshReasoningView
	 * 
	 */
	public MeshReasoningViewControl(MeshCas cas, ArrayList<MeshAnalyser> analyser,
			MeshReasoningView view) {
		this.view = view;
		this.cas = cas;
		setPreferredSize(new Dimension(300, 600));

		setLayout(new BorderLayout());
		this.analyser = analyser;

		accordion = new MeshCasAccordion(cas);
		analyserListModel = new DefaultListModel();

		analyserList = new JList(analyserListModel);
		updateAnalyserList();

		JScrollPane scrollAccordion = new JScrollPane(accordion,
				ScrollPaneConstants.VERTICAL_SCROLLBAR_AS_NEEDED,
				ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
		this.add(scrollAccordion, BorderLayout.CENTER);

		JScrollPane listScroller = new JScrollPane(analyserList);
		listScroller.setPreferredSize(new Dimension(250, 100));
		analyserList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		analyserList.setLayoutOrientation(JList.VERTICAL);

		JPanel bottomPnl = new JPanel(new BorderLayout());

		bottomPnl.add(listScroller, BorderLayout.CENTER);

		JPanel drawSettingsContainer = new JPanel(new BorderLayout());
		drawSettingsPanel = new DrawSettingsPanel(cas, view);
		drawSettingsContainer.add(drawSettingsPanel, BorderLayout.CENTER);
		btnSave = new JButton("Save image");
		btnSave.addActionListener(this);
		drawSettingsContainer.add(btnSave, BorderLayout.SOUTH);

		bottomPnl.add(drawSettingsContainer, BorderLayout.SOUTH);

		this.add(bottomPnl, BorderLayout.SOUTH);

		JPanel topPnl = new JPanel(new GridBagLayout());
		cbxShowMesh = new JCheckBox("Show mesh");
		cbxShowMesh.addActionListener(this);
		cbxShowMesh.setSelected(true);
		GridBagConstraints c = new GridBagConstraints();
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 0;
		c.gridy = 0;
		topPnl.add(cbxShowMesh, c);

		this.add(topPnl, BorderLayout.NORTH);

		Timer tim = new Timer();
		tim.scheduleAtFixedRate(new TimerTask() {

			@Override
			public void run() {
				updateAnalyserList();

			}
		}, 1000, 500);

	}

	@Override
	public void actionPerformed(ActionEvent e) {
		if (e.getSource() == cbxShowMesh)
			cas.setDrawMesh(cbxShowMesh.isSelected());
		else if (e.getSource() == btnSave) {
			String str = (String) JOptionPane.showInputDialog(null, "Enter a filename ",
					"Filename", JOptionPane.INFORMATION_MESSAGE, null, null, defaultImageFilename);
			if (str != null)
				view.saveImage(str);
		}

	}

	/**
	 * @return the analyser
	 */
	public ArrayList<MeshAnalyser> getAnalyser() {
		return analyser;
	}

	/**
	 * @return the cas
	 */
	public MeshCas getCas() {
		return cas;
	}

	public void setDefaultImageFilename(String s) {
		defaultImageFilename = s;
	}

	/**
	 * Shows the selected annotation in the accordion view if only one is selected. Otherwise the
	 * selection will be cleared.
	 * 
	 * @param selectedAnnotations
	 *            list of currently selected annotations
	 */
	public void showSelectedAnnotation(HashSet<MeshAnnotation> selectedAnnotations) {
		if (selectedAnnotations.size() == 1) {
			accordion.setSelectedAnnotation(selectedAnnotations.iterator().next());
		} else
			accordion.setSelectedAnnotation(null);
	}

	/**
	 * Updates the status in the analyser list (the progress and current duration)
	 */
	void updateAnalyserList() {
		int i;
		synchronized (analyser) {
			for (i = 0; i < analyser.size(); i++) {
				if (analyserListModel.getSize() <= i)
					analyserListModel.addElement(analyser.get(i).getNameAndProgress());
				else
					analyserListModel.setElementAt(analyser.get(i).getNameAndProgress(), i);
			}
			for (i = analyser.size(); i < analyserListModel.getSize(); i++)
				analyserListModel.remove(analyser.size());
		}
	}

}
