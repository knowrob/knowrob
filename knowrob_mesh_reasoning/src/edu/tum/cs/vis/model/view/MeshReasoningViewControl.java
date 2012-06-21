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
import javax.swing.JCheckBox;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.ListSelectionModel;
import javax.swing.ScrollPaneConstants;

import edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer;
import edu.tum.cs.vis.model.uima.cas.MeshCas;

public final class MeshReasoningViewControl extends JPanel implements ActionListener {

	private final MeshReasoningView			view;

	private final ArrayList<MeshAnalyzer>	analyzer;
	private final MeshCasAccordion			accordion;

	private final JList						analyzerList;
	private final DefaultListModel			analyzerListModel;

	private final JCheckBox					cbxShowMesh;
	private final MeshCas					cas;

	/**
	 * Default constructor
	 * 
	 * @param parent
	 *            Applet for which this control panel is
	 */
	public MeshReasoningViewControl(MeshReasoningView parent, MeshCas cas,
			ArrayList<MeshAnalyzer> analyzer) {
		view = parent;
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

		this.add(listScroller, BorderLayout.SOUTH);

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

	}

	private void updateAnalyzerList() {
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
