/**
 * 
 */
package edu.tum.cs.vis.model.view;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.util.ArrayList;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.ButtonGroup;
import javax.swing.DefaultListModel;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JList;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JScrollPane;
import javax.swing.ListSelectionModel;
import javax.swing.ScrollPaneConstants;

import edu.tum.cs.vis.model.uima.analyser.MeshAnalyser;
import edu.tum.cs.vis.model.uima.annotation.DrawableAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.DrawType;
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
	private final JList<String>				analyserList;
	/**
	 * List model for analyserList
	 */
	private final DefaultListModel<String>	analyserListModel;

	/**
	 * Panel which contains controls which influence drawing of model
	 */
	private final DrawSettingsPanel			drawSettingsPanel;

	/**
	 * check box to enable or disable drawing of main mesh (base data)
	 */
	private final JCheckBox					cbxShowMesh;

	/**
	 * Button to open ControlInfoDialog
	 */
	private final JButton					btnControlInfo;

	/**
	 * Radio button for draw type filled
	 */
	private final JRadioButton				rbnDrawFill;

	/**
	 * Radio button for draw type lines
	 */
	private final JRadioButton				rbnDrawLines;

	/**
	 * Radio button for draw type points
	 */
	private final JRadioButton				rbnDrawPoints;

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
	 * default filename to save image (screenshot)
	 */
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
		analyserListModel = new DefaultListModel<String>();

		analyserList = new JList<String>(analyserListModel);
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

		GridBagConstraints c = new GridBagConstraints();
		JPanel topPnl1 = new JPanel(new GridBagLayout());
		JPanel topPnl2 = new JPanel(new GridBagLayout());
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 0;
		c.gridy = 0;
		topPnl.add(topPnl1, c);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 0;
		c.gridy = 1;
		topPnl.add(topPnl2, c);

		cbxShowMesh = new JCheckBox("Show mesh");
		cbxShowMesh.addActionListener(this);
		cbxShowMesh.setSelected(true);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 0;
		c.gridy = 0;
		topPnl1.add(cbxShowMesh, c);

		btnControlInfo = new JButton("control info");
		btnControlInfo.addActionListener(this);
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 1;
		c.gridy = 0;
		topPnl1.add(btnControlInfo, c);

		// Create the radio buttons.
		rbnDrawFill = new JRadioButton("Fill");
		rbnDrawFill.setMnemonic(KeyEvent.VK_1);
		rbnDrawFill.setSelected(true);
		rbnDrawFill.addActionListener(this);
		rbnDrawLines = new JRadioButton("Lines");
		rbnDrawLines.setMnemonic(KeyEvent.VK_1);
		rbnDrawLines.setSelected(true);
		rbnDrawLines.addActionListener(this);
		rbnDrawPoints = new JRadioButton("Points");
		rbnDrawPoints.setMnemonic(KeyEvent.VK_1);
		rbnDrawPoints.setSelected(true);
		rbnDrawPoints.addActionListener(this);

		// Group the radio buttons.
		ButtonGroup group = new ButtonGroup();
		group.add(rbnDrawFill);
		group.add(rbnDrawLines);
		group.add(rbnDrawPoints);
		c = new GridBagConstraints();
		c.fill = GridBagConstraints.HORIZONTAL;
		c.gridx = 0;
		c.gridy = 1;
		topPnl2.add(rbnDrawFill, c);
		c.gridx = 1;
		c.gridy = 1;
		topPnl2.add(rbnDrawLines, c);
		c.gridx = 2;
		c.gridy = 1;
		topPnl2.add(rbnDrawPoints, c);

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
		if (e.getSource() == btnControlInfo) {
			@SuppressWarnings("unused")
			ControlInfoDialog cid = new ControlInfoDialog(null);
		} else if (e.getSource() == btnSave) {
			String str = (String) JOptionPane.showInputDialog(null, "Enter a filename ",
					"Filename", JOptionPane.INFORMATION_MESSAGE, null, null, defaultImageFilename);
			if (str != null)
				view.saveImage(str);
		} else if (e.getSource() == rbnDrawFill)
			view.setDrawType(DrawType.FILL);
		else if (e.getSource() == rbnDrawLines)
			view.setDrawType(DrawType.LINES);
		else if (e.getSource() == rbnDrawPoints)
			view.setDrawType(DrawType.POINTS);
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

	/**
	 * Get default image filename set by corresponding setter.
	 * 
	 * @return default image filename
	 */
	public String getDefaultImageFilename() {
		return defaultImageFilename;
	}

	/**
	 * Sets default image file name to save image (screenshot)
	 * 
	 * @param s
	 *            new file name
	 */
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
	public void showSelectedAnnotation(Map<DrawableAnnotation, Color> selectedAnnotations) {
		if (selectedAnnotations.size() == 1) {
			accordion.setSelectedAnnotation(selectedAnnotations.keySet().iterator().next());
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
