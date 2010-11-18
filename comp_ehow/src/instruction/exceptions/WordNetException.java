package instruction.exceptions;

import instruction.semanticObjects.Word;


public class WordNetException extends Exception {

	private static final long serialVersionUID = -1828674915260265308L;

	private Word w = null;
	
	public WordNetException(String msg) {
		super(msg);
	}
	
	public WordNetException(Word w) {
		this.w = w;
	}
	
	public Word getWord() {
		return w;
	}
	
}
