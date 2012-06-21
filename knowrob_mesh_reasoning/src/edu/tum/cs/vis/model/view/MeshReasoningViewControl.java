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

import org.apache.log4j.Logger;

import edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer;
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
	 * Log4J Logger
	 */
	private static Logger					logger				= Logger.getLogger(MeshReasoningViewControl.class);

	/**
	 * auto generated
	 */
	private static final long				serialVersionUID	= -2013371551022209950L;

	/**
	 * list of all analyzers
	 */
	private final ArrayList<MeshAnalyzer>	analyzer;
	/**
	 * The accordion component which contains controls for each different annotation type
	 */
	private final MeshCasAccordion			accordion;

	/**
	 * List which shows the current progress of all analyzers
	 */
	private final JList						analyzerList;
	/**
	 * List model for analyzerList
	 */
	private final DefaultListModel			analyzerListModel;

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

	/**
	 * Default constructor
	 * 
	 * @param cas
	 *            cas for which the control is
	 * @param analyzer
	 *            list of analyzers used
	 * @param view
	 *            the parent MeshReasoningView
	 * 
	 */
	public MeshReasoningViewControl(MeshCas cas, ArrayList<MeshAnalyzer> analyzer,
			MeshReasoningView view) {
		this.view = view;
		this.cas = cas;
		setPreferredSize(new Dimension(300, 600));

		setLayout(new BorderLayout());
		this.analyzer = analyzer;

		accordion = new MeshCasAccordion(cas);
		analyzerListModel = new DefaultListModel();

		analyzerList = new JList(analyzerListModel);
		updateAnalyzerList();

		JScrollPane scrollAccordion = new JScrollPane(accordion,
				ScrollPaneConstants.VERTICAL_SCROLLBAR_AS_NEEDED,
				ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
		this.add(scrollAccordion, BorderLayout.CENTER);

		JScrollPane listScroller = new JScrollPane(analyzerList);
		listScroller.setPreferredSize(new Dimension(250, 100));
		analyzerList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		analyzerList.setLayoutOrientation(JList.VERTICAL);

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
				updateAnalyzerList();

			}
		}, 1000, 500);

	}

	@Override
	public void actionPerformed(ActionEvent e) {
		if (e.getSource() == cbxShowMesh)
			cas.setDrawMesh(cbxShowMesh.isSelected());
		else if (e.getSource() == btnSave) {
			String str = JOptionPane.showInputDialog(null, "Enter a filename ", "Filename",
					JOptionPane.INFORMATION_MESSAGE);
			if (str != null)
				view.saveImage(str);
		}

	}

	/**
	 * Shows the selected annotation in the accordion view if only one is selected. Otherwise the
	 * selection will be cleared.
	 * 
	 * @param selectedAnnotations
	 *            list of currently selected annotations
	 */
	public void showSelectedAnnotation(ArrayList<MeshAnnotation> selectedAnnotations) {
		if (selectedAnnotations.size() == 1) {
			accordion.setSelectedAnnotation(selectedAnnotations.get(0));
		} else
			accordion.setSelectedAnnotation(null);
	}

	/**
	 * Updates the status in the analyzer list (the progress and current duration)
	 */
	void updateAnalyzerList() {
		int i;
		synchronized (analyzer) {
			for (i = 0; i < analyzer.size(); i++) {
				if (analyzerListModel.getSize() <= i)
					analyzerListModel.addElement(analyzer.get(i).getNameAndProgress());
				else
					analyzerListModel.setElementAt(analyzer.get(i).getNameAndProgress(), i);
			}
			for (i = analyzer.size(); i < analyzerListModel.getSize(); i++)
				analyzerListModel.remove(analyzer.size());
		}
	}

}
