package instruction.importer;

import instruction.semanticObjects.Word;

public interface AddCycMappingListener {

	/**
	 * Returns a string array where [1] is the synset and [2] is the concept to it.
	 * @param word
	 * @return
	 */
	public String[] getCycMappingForWord(Word word);
	
}
