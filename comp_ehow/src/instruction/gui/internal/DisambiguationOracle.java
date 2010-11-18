package instruction.gui.internal;

import java.util.List;
import javax.swing.JPanel;
import instruction.disambiguator.IDisambiguationOracle;
import instruction.gui.dialog.ConceptSelectionDlg;
import instruction.semanticObjects.Instruction;

public class DisambiguationOracle implements IDisambiguationOracle {

	private JPanel parent = null;
	
	public DisambiguationOracle(JPanel parent) {
		this.parent = parent;
	}
	
	public String retrieveMeaningOfWord(String word, List<String> meanings,
			Instruction instruction) {
		
		ConceptSelectionDlg dlg = new ConceptSelectionDlg(null);
		
		dlg.getTextInstruction().setText("<html><p>" + instruction.getNLSentence().replaceAll(word, "<u>" + word + "</u>") + "</p></html>");
	//	dlg.getLabelWord().setText("\"" + word + "\"");
		dlg.getConceptList().setListData(meanings.toArray(new String[0]));
		dlg.setLocationRelativeTo( parent );
		dlg.setModal(true);
		
		dlg.setVisible(true);
		
		if (dlg.getConceptList().getSelectedValue() == null)
			return null;
		else
			return (String) dlg.getConceptList().getSelectedValue();
	}

}
