package instruction.exceptions;

import instruction.semanticObjects.Word;

public class CycMappingException extends Exception {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	Word w;
	
	public CycMappingException(Word w) {
		this.w = w;
	}
	
	public Word getWord() {
		return w;
	}
	
}
