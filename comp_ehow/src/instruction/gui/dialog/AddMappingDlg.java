package instruction.gui.dialog;

import instruction.semanticObjects.Word;
import instruction.wordnet.WordNetRDF2;

import java.awt.Dimension;
import java.awt.Frame;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.DefaultListModel;
import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextField;
import javax.swing.ListSelectionModel;
import javax.swing.ScrollPaneConstants;
import javax.swing.SpringLayout;

public class AddMappingDlg extends JDialog {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	JButton jButton = null;
	JPanel jContentPane = null;
	JTextField text = null;
	JList sensesList = null;
	
	Word w = null;
	private String synset = null;
	private String concept = null;
	
	DefaultListModel listModel = new DefaultListModel();

	public AddMappingDlg(Frame owner, Word w) {
		super(owner);
		this.w = w;
		initialize();
	}

	/**
	 * This method initializes this
	 * 
	 * @return void
	 */
	private void initialize() {
		this.setContentPane(getJContentPane());
		this.setTitle("Add Cyc-Mapping for Word Sense");
		setSize(400, 465);
	}

	/**
	 * This method initializes jContentPane
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJContentPane() {
		if (jContentPane == null) {
			jContentPane = new JPanel();

			SpringLayout layout = new SpringLayout();
			jContentPane.setLayout(layout);

			JLabel label1 = new JLabel();
			label1
					.setText("<html><p>Choose the appropriate word sense from the WordNet ontology.<br><br>The word under consideration is<br><br><p align=\"center\">"
							+ w.getLabel() + " (word stem is: " + WordNetRDF2.getWordStem(w.getLabel(), WordNetRDF2.convertPOS(w.getType())) + ")"
							+ "</p><br>These are possible senses offered by WordNet:</p></html>");

			jContentPane.add(label1);

			ArrayList<String> synsets = WordNetRDF2.getSynsets(w.getLabel(),  WordNetRDF2.convertPOS(w.getType()));

			GlossaryEntry[] entries = new GlossaryEntry[synsets.size()];
			for (int i = 0; i < entries.length; i++) {
				entries[i] = new GlossaryEntry(synsets.get(i), WordNetRDF2.getGlossaryForSynset(synsets.get(i)));
			}

			sensesList = new JList(entries);
			sensesList.setVisibleRowCount(4);
			sensesList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
			sensesList.setLayoutOrientation(JList.VERTICAL);
		
			JScrollPane scroller = new JScrollPane(sensesList);
			scroller
					.setVerticalScrollBarPolicy(ScrollPaneConstants.VERTICAL_SCROLLBAR_ALWAYS);
			scroller
					.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_ALWAYS);
			scroller.setPreferredSize(new Dimension(-1,120));
			

			JButton ok = new JButton();
			ok.setText("OK");
			ok.addActionListener(new ActionListener() {

				public void actionPerformed(ActionEvent arg0) {
					AddMappingDlg.this.concept = AddMappingDlg.this.text.getText();
					AddMappingDlg.this.synset = ((AddMappingDlg.GlossaryEntry)sensesList.getSelectedValue()).getSynset();
					AddMappingDlg.this.setVisible(false);
				}

			});
			JLabel label2 = new JLabel();
			label2
					.setText("<html><p>Now type the appropriate Cyc Concept the sense "
							+ "<br>shall be mapped to:</p></html>");

			JLabel label3 = new JLabel();
			label3
					.setText("<html><p>The Mapping will be added to the Cyc ontology persistently. "
							+ "If you press cancel, the whole instruction will be discarded.</p></html>");

			text = new JTextField();
			text.setText("Thing");

			JButton cancel = new JButton();
			cancel.setText("Cancel");
			cancel.addActionListener(new ActionListener() {
				
				@Override
				public void actionPerformed(ActionEvent e) {
					AddMappingDlg.this.setVisible(false);
					
				}
			});

			layout.putConstraint(SpringLayout.NORTH, label1, 10,
					SpringLayout.NORTH, jContentPane);
			layout.putConstraint(SpringLayout.NORTH, scroller, 10,
					SpringLayout.SOUTH, label1);
			layout.putConstraint(SpringLayout.EAST, label1, -10,
					SpringLayout.EAST, jContentPane);
			layout.putConstraint(SpringLayout.WEST, label1, 10,
					SpringLayout.WEST, jContentPane);

			layout.putConstraint(SpringLayout.EAST, scroller, -10,
					SpringLayout.EAST, jContentPane);
			layout.putConstraint(SpringLayout.WEST, scroller, 10,
					SpringLayout.WEST, jContentPane);

			layout.putConstraint(SpringLayout.SOUTH, ok, -10,
					SpringLayout.SOUTH, jContentPane);
			layout.putConstraint(SpringLayout.WEST, ok, 10, SpringLayout.WEST,
					jContentPane);

			layout.putConstraint(SpringLayout.NORTH, label2, 10,
					SpringLayout.SOUTH, scroller);
			layout.putConstraint(SpringLayout.WEST, label2, 10,
					SpringLayout.WEST, jContentPane);
			layout.putConstraint(SpringLayout.EAST, label1, -10,
					SpringLayout.EAST, jContentPane);

			layout.putConstraint(SpringLayout.NORTH, text, 20,
					SpringLayout.SOUTH, label2);
			layout.putConstraint(SpringLayout.WEST, text, 10,
					SpringLayout.WEST, jContentPane);
			layout.putConstraint(SpringLayout.EAST, text, -10,
					SpringLayout.EAST, jContentPane);
			layout.putConstraint(SpringLayout.SOUTH, text, -20,
					SpringLayout.NORTH, label3);
			layout.putConstraint(SpringLayout.WEST, label3, 10,
					SpringLayout.NORTH, jContentPane);
			layout.putConstraint(SpringLayout.EAST, label3, 10,
					SpringLayout.EAST, jContentPane);
			layout.putConstraint(SpringLayout.SOUTH, label3, -10,
					SpringLayout.NORTH, ok);

			layout.putConstraint(SpringLayout.SOUTH, cancel, -10,
					SpringLayout.SOUTH, jContentPane);
			layout.putConstraint(SpringLayout.WEST, cancel, 10,
					SpringLayout.EAST, ok);

			jContentPane.add(cancel);
			jContentPane.add(label3);
			jContentPane.add(label2);
			jContentPane.add(scroller);
			jContentPane.add(ok);
			jContentPane.add(text);
			
			
		}

		return jContentPane;
	}

	public String getConcept() {
		return concept;
	}
	
	public String getSynset() {
		return synset;
	}
	
	private class GlossaryEntry {

		private String entry;
		private String synset;

		public GlossaryEntry(String synset, String entry) {
			this.entry = entry;
			this.synset = synset;
		}
		
		

		public String getSynset() {
			return synset;
		}

		@Override
		public String toString() {
			return entry;
		}

	}

	public static void main(String[] args) {

		AddMappingDlg dlg = new AddMappingDlg(null, new Word(Word.TYPE_PAST_PARTICIPLE, "had"));
		dlg.setModal(true);
		dlg.setPreferredSize(null);
		dlg.pack();
		dlg.setVisible(true);
	}
}
