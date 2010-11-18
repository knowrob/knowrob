package instruction.exceptions;

import instruction.semanticObjects.Word;

public class AmbiguityException extends Exception {
	
	private static final long serialVersionUID = -6383964812984392010L;
	private Word ambiguousWord;
	
	public AmbiguityException(String msg) {
		super(msg);
	}
	
	public AmbiguityException(String msg, Word word) {
		super(msg);
		ambiguousWord = word;
	}
	
	/**
	 * Returns the word of which the meanings could not be disambiguated
	 * @return
	 */
	public Word getAmbiguousWord() {
		return ambiguousWord;
	}
}
