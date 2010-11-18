package instruction.gui.dialog;

import java.awt.Dimension;
import java.awt.Font;
import java.awt.Frame;
import java.awt.SystemColor;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.ListSelectionModel;
import javax.swing.SpringLayout;

public class ConceptSelectionDlg extends JDialog {

	private static final long serialVersionUID = 1L;
	private JPanel jContentPane = null;
	private JLabel jLabel = null;
	private JLabel labelWord = null;
	private JButton jButton = null;
	private JLabel textInstruction = null;
	private JList conceptList = null;
	/**
	 * @param owner
	 */
	public ConceptSelectionDlg ( Frame owner ) {

		super( owner );
		initialize();
	}

	/**
	 * This method initializes this
	 * 
	 * @return void
	 */
	private void initialize() {

		
		this.setContentPane(getJContentPane());
		this.setTitle("Disambiguator Training");
		this.setSize(650,250);
	}
	
	/**
	 * This method initializes jContentPane
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJContentPane() {

		if ( jContentPane == null ) {
			
			jContentPane = new JPanel();
			SpringLayout layout = new SpringLayout();
			jContentPane.setLayout(layout);
			
			getConceptList();
			getTextInstruction();
			JButton button = getJButton();
			jLabel = new JLabel();
			jLabel.setText("Please select the correct sense of the underlined word in the following instruction:");
			jContentPane.add(jLabel);
			
			labelWord = new JLabel();
			labelWord.setText("");
			
			
			layout.putConstraint(SpringLayout.NORTH, jLabel, 10, SpringLayout.NORTH, jContentPane);
			layout.putConstraint(SpringLayout.WEST, jLabel, 10, SpringLayout.WEST, jContentPane);
			layout.putConstraint(SpringLayout.EAST, jLabel, -10, SpringLayout.EAST, jContentPane);
			
			layout.putConstraint(SpringLayout.NORTH, textInstruction, 0, SpringLayout.SOUTH, jLabel);
			layout.putConstraint(SpringLayout.WEST, textInstruction, 10, SpringLayout.WEST, jContentPane);
			layout.putConstraint(SpringLayout.EAST, textInstruction, -10, SpringLayout.EAST, jContentPane);
			
			layout.putConstraint(SpringLayout.NORTH, conceptList, 0, SpringLayout.SOUTH, textInstruction);
			layout.putConstraint(SpringLayout.WEST, conceptList, 10, SpringLayout.WEST, jContentPane);
			layout.putConstraint(SpringLayout.EAST, conceptList, -10, SpringLayout.EAST, jContentPane);
			
			layout.putConstraint(SpringLayout.NORTH, button, 20, SpringLayout.SOUTH, conceptList);
			layout.putConstraint(SpringLayout.SOUTH, button, -10, SpringLayout.SOUTH, jContentPane);
			layout.putConstraint(SpringLayout.WEST, button, 10, SpringLayout.WEST, jContentPane);
			
			
			
			jContentPane.add(labelWord);
			jContentPane.add(button);
			jContentPane.add(textInstruction);
			jContentPane.add(conceptList);
			
		}
		return jContentPane;
	}

	/**
	 * This method initializes jButton	
	 * 	
	 * @return javax.swing.JButton	
	 */
	private JButton getJButton() {
	
		if ( jButton == null ) {
			jButton = new JButton();
			jButton.setText("OK");
			jButton.setPreferredSize(new Dimension(100, 30));
			jButton.addActionListener(new ActionListener() {

				
				public void actionPerformed(ActionEvent arg0) {
					ConceptSelectionDlg.this.setVisible(false);
				}
				
			});
		}
		return jButton;
	}

	/**
	 * This method initializes textInstruction	
	 * 	
	 * @return javax.swing.JTextField	
	 */
	public JLabel getTextInstruction() {
		if (textInstruction == null) {
			textInstruction = new JLabel();
			textInstruction.setText("Instruction");
			textInstruction.setBackground(SystemColor.control);
			textInstruction.setFont(new Font("Dialog", Font.BOLD, 12));
			textInstruction.setPreferredSize(new Dimension(61, 50));
			textInstruction.setBorder(BorderFactory.createEmptyBorder());
		}
		return textInstruction;
	}

	/**
	 * This method initializes conceptList	
	 * 	
	 * @return javax.swing.JList	
	 */
	public JList getConceptList() {
		if (conceptList == null) {
			conceptList = new JList();
			conceptList.setBorder(BorderFactory.createEtchedBorder());
			conceptList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
			conceptList.setLayoutOrientation(JList.VERTICAL);
			conceptList.setPreferredSize(new Dimension(0,100));
		}
		return conceptList;
	}
	
	public JLabel getLabelWord() {
		return labelWord;
	}

}  //  @jve:decl-index=0:visual-constraint="124,82"
