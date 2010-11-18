package instruction.gui;

import instruction.gui.dialog.AddMappingDlg;
import instruction.importer.AddCycMappingListener;
import instruction.semanticObjects.Word;

public class AddMappingListener implements AddCycMappingListener {

	String synset = null;
	String concept = null;
	
	@Override
	public String[] getCycMappingForWord(Word word) {
		AddMappingDlg dlg = new AddMappingDlg(null, word);
		dlg.setModal( true );		
		dlg.setVisible( true );
		
		synset = dlg.getSynset();
		concept = dlg.getConcept();
		if (synset == null)
			return null;
		return new String[] {synset, concept};
	}

}
