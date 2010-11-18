package instruction.gui;

import instruction.gui.dialog.InstructionProgressDlg;
import instruction.configuration.ConfigurationManager;
import instruction.gui.internal.DisambiguationOracle;
import instruction.gui.internal.PlanImporterWrapper;
import instruction.gui.tab.BrowserTab;
import instruction.gui.tab.CycViewTab;
import instruction.gui.tab.InternalViewTab;
import instruction.gui.tab.OnlineSearchPanel;
import instruction.gui.tab.RPLViewTab;
import instruction.gui.tab.SearchViewTab;
import instruction.gui.tab.TreeViewTab;
import instruction.opencyc.OpenCyc;
import instruction.wrapper.LocalFileWrapper;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JTabbedPane;
import javax.swing.UIManager;


public class EHowInstructionPanel extends JPanel {

	private static final long serialVersionUID = 7862127773357644721L;

	JPanel[] tabs = { new SearchViewTab(), new BrowserTab(), new TreeViewTab(),
			new InternalViewTab(), new CycViewTab(), new RPLViewTab() };

	JPanel me = this;
	JComponent bar = null;
	JTabbedPane tabComponent = null;

	public void init() {

		ConfigurationManager.loadSettings();

		try {
			PlanImporterWrapper.getImporter().initialize();
		} catch (Exception e1) {
			e1.printStackTrace();
		}
		
		tabComponent = new JTabbedPane();
		setLayout(new BorderLayout(0,0));

		
		for (JPanel tab : tabs) {

			try {
				tabComponent.addTab(tab.getClass().getField("TITLE").get(null)
						.toString(), null, tab);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}

		add(tabComponent, BorderLayout.CENTER);
		
		
		for (int i = 1; i < tabs.length; i++) {
			tabComponent.setEnabledAt(i, false);
		}

		//tabComponent.setEnabledAt(5, true);

		// *************************************************************************
		// Initialize the bar at the bottom
		// *************************************************************************
		bar = new JPanel();
		
		JButton next = new JButton("Next >");

		next.addActionListener(new ActionListener() {

			public void actionPerformed(ActionEvent arg0) {

				int idx = tabComponent.getSelectedIndex();

				if (idx == 0) {
					new Thread() {
						public void run() {

						//	PlanImporterWrapper.getImporter().addAddCycMappingListener(addMapping);
							try {
								InstructionProgressDlg dlg = new InstructionProgressDlg(
										null);
								dlg
										.setLocationRelativeTo(EHowInstructionPanel.this);
								dlg.setModal(false);
								dlg.setVisible(true);

								PlanImporterWrapper.getImporter()
										.addProgressListener(dlg);
								PlanImporterWrapper.getImporter().initialize();
								PlanImporterWrapper.getImporter().setAddMappingListener(new AddMappingListener());
								Map<String, List<String>> mappings = ConfigurationManager
										.getMappings();
								Set<String> synsets = mappings.keySet();
								for (Iterator<String> i = synsets.iterator(); i
										.hasNext();) {
									String synset = i.next();
									List<String> concepts = mappings
											.get(synset);
									for (Iterator<String> j = concepts
											.iterator(); j.hasNext();) {
										OpenCyc.getInstance().addMapping(
												synset, j.next());
									}
								}
								PlanImporterWrapper
										.getImporter()
										.getDisambiguator()
										.load(
												ConfigurationManager
														.getPathDisambiguator());
								PlanImporterWrapper
										.getImporter()
										.getDisambiguator()
										.setDisambiguationOracle(
												new DisambiguationOracle(
														EHowInstructionPanel.this));
								PlanImporterWrapper
										.getImporter()
										.getDisambiguator()
										.setRunMode(
												ConfigurationManager
														.getDisambiguatorMode());
								LocalFileWrapper wrp = new LocalFileWrapper();
								String howto = ((SearchViewTab) tabComponent
										.getSelectedComponent()).getHowtoPath();
								File dummy = new File(howto);
								if (dummy.exists())
									wrp.load(howto);
								else {
									howto = howto.replaceAll(" ", "_");
									howto = ConfigurationManager
											.getPathHowtos()
											+ File.separator + howto;
									wrp.load(howto);
								}

								PlanImporterWrapper.getImporter().setWrapper(
										wrp);

								dlg.setVisible(false);
								PlanImporterWrapper.getImporter()
										.removeProgressListener(dlg);
								tabComponent.setSelectedIndex((tabComponent
										.getSelectedIndex() + 1)
										% tabs.length);
								tabComponent.setEnabledAt(1, true);
							} catch (Exception e) {
								JOptionPane.showMessageDialog(
										EHowInstructionPanel.this, e
												.getMessage(), "Error",
										JOptionPane.ERROR_MESSAGE);
								e.printStackTrace();
							}
						}
					}.start();
				}

				else if (idx == 1) {
					new Thread() {
						public void run() {

							try {
								InstructionProgressDlg dlg = new InstructionProgressDlg(
										null);
								dlg
										.setLocationRelativeTo(EHowInstructionPanel.this);
								dlg.setModal(false);
								dlg.setVisible(true);
								PlanImporterWrapper.getImporter()
										.addProgressListener(dlg);

								PlanImporterWrapper.getImporter()
										.parseInstructions();

								dlg.setVisible(false);
								PlanImporterWrapper.getImporter()
										.removeProgressListener(dlg);
								tabComponent.setSelectedIndex((tabComponent
										.getSelectedIndex() + 1)
										% tabs.length);
								tabComponent.setEnabledAt(2, true);
							} catch (Exception e) {
								JOptionPane.showMessageDialog(
										EHowInstructionPanel.this, e
												.getMessage(), "Error",
										JOptionPane.ERROR_MESSAGE);
								e.printStackTrace();
							}
						}
					}.start();
				}

				else if (idx == 2) {
					new Thread() {
						public void run() {

							try {
								InstructionProgressDlg dlg = new InstructionProgressDlg(
										null);
								dlg
										.setLocationRelativeTo(EHowInstructionPanel.this);
								dlg.setModal(false);
								dlg.setVisible(true);
								PlanImporterWrapper.getImporter()
										.addProgressListener(dlg);

								PlanImporterWrapper.getImporter()
										.recognizeAndDisambiguateInstructions();

								dlg.setVisible(false);
								PlanImporterWrapper.getImporter()
										.removeProgressListener(dlg);
								tabComponent.setSelectedIndex((tabComponent
										.getSelectedIndex() + 1)
										% tabs.length);
								tabComponent.setEnabledAt(3, true);
							} catch (Exception e) {
								JOptionPane.showMessageDialog(
										EHowInstructionPanel.this, e
												.getMessage(), "Error",
										JOptionPane.ERROR_MESSAGE);
								e.printStackTrace();
							}
						}
					}.start();
				}

				else if (idx == 3) {
					new Thread() {
						public void run() {

							try {
								InstructionProgressDlg dlg = new InstructionProgressDlg(
										null);
								dlg
										.setLocationRelativeTo(EHowInstructionPanel.this);
								dlg.setModal(false);
								dlg.setVisible(true);
								PlanImporterWrapper.getImporter()
										.addProgressListener(dlg);

								PlanImporterWrapper.getImporter()
										.convert2CycAssertions();

								PlanImporterWrapper.getImporter()
										.removeProgressListener(dlg);
								dlg.setVisible(false);
								tabComponent.setSelectedIndex((tabComponent
										.getSelectedIndex() + 1)
										% tabs.length);
								tabComponent.setEnabledAt(4, true);
							} catch (Exception e) {
								JOptionPane.showMessageDialog(
										EHowInstructionPanel.this, e
												.getMessage(), "Error",
										JOptionPane.ERROR_MESSAGE);
								e.printStackTrace();
							}
						}
					}.start();
				}

				else if (idx == 4) {
					new Thread() {
						public void run() {

							try {
								InstructionProgressDlg dlg = new InstructionProgressDlg(
										null);
								dlg
										.setLocationRelativeTo(EHowInstructionPanel.this);
								dlg.setModal(false);
								dlg.setVisible(true);
								PlanImporterWrapper.getImporter()
										.addProgressListener(dlg);

								PlanImporterWrapper.getImporter()
										.generateRPLPlan();

								PlanImporterWrapper.getImporter()
										.removeProgressListener(dlg);
								dlg.setVisible(false);
								tabComponent.setSelectedIndex((tabComponent
										.getSelectedIndex() + 1)
										% tabs.length);
								tabComponent.setEnabledAt(5, true);
							} catch (Exception e) {
								JOptionPane.showMessageDialog(
										EHowInstructionPanel.this, e
												.getMessage(), "Error",
										JOptionPane.ERROR_MESSAGE);
								e.printStackTrace();
							}
						}
					}.start();
				}
			}

		});

		bar.add(next);
		add(bar, BorderLayout.SOUTH);
		setMinimumSize(new Dimension(1100, 800));

	}

}
