package instruction.semanticObjects;

import java.util.ArrayList;

public class Quantifier extends SemanticObject {

	private ArrayList<Word> alternatives = new ArrayList<Word>();
	private Word measure = new Word();

	public Quantifier () {

	}
	
	public void setMeasure(Word measure) {
		this.measure = measure;
	}
	
	public Word getMeasure() {
		return measure;
	}

	public ArrayList<Word> getAlternatives() {

		return alternatives;
	}
	
	public void setAlternatives(ArrayList<Word> alt) {
		alternatives = alt;
	}
	
	public String toString() {
		String str = "({";
		for (int i = 0; i < alternatives.size(); i++) {
			str += alternatives.get( i );
			if (i < alternatives.size() - 1)
				str += ",";
		}
		str += "}," + measure + ")";
		return str;
	}

	public boolean equals( Object q ) {
		if (! (q instanceof Quantifier))
			return false;
		return alternatives.equals( ((Quantifier) q).getAlternatives() );
	}
}
