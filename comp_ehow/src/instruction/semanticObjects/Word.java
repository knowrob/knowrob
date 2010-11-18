package instruction.semanticObjects;

import java.util.ArrayList;
import java.util.Iterator;

/**
 * 
 * @author Daniel Nyga
 * 
 */
public class Word extends SemanticObject {

	public static final int TYPE_UNKNOWN = - 1;
	public static final int TYPE_NOUN = 0;
	public static final int TYPE_VERB_INFINITIVE = 1;
	public static final int TYPE_ADJ = 2;
	public static final int TYPE_ADV = 3;
	public static final int TYPE_PARTICLE = 4;
	public static final int TYPE_NUMBER = 5;
	public static final int TYPE_CONJ = 7;
	public static final int TYPE_PRONOUN = 8;
	public static final int TYPE_PREPOSITION = 9;
	public static final int TYPE_ARTICLE = 10;
	public static final int TYPE_PUNCTUATION = 11;
	public static final int TYPE_PAST_PARTICIPLE = 12;
	public static final int TYPE_GERUND = 13;

	/**
	 * Specifies the type of the Word. One of the <code>TYPE_</code>-Fields
	 */
	private int type;

	/**
	 * The label of the Word
	 */
	private String label;

	/**
	 * A List of WordNet-SynSet-IDs.<br>
	 * <b>Is not automatically filled!</b><br>
	 * Use <code>WordNet.getSynIDs() method to retrieve them!</code>
	 */
	private ArrayList<String> synSetIDs = new ArrayList<String>();

	/**
	 * A List of Cyc-Concepts associated with the SynSets of
	 * <code>synSetIDs</code><br>
	 * <b>Is not automatically filled!</b>
	 */
	private ArrayList<String> cycConcepts = new ArrayList<String>();

	/**
	 * Constructs a new <code>Word</code> with empty label and unknown type
	 */
	public Word () {

		type = TYPE_UNKNOWN;
		label = "";
	}

	public Word ( int type, String label ) {

		this.type = type;
		this.label = label;
	}

	public ArrayList<String> getSynSetIDs() {

		return synSetIDs;
	}

	public ArrayList<String> getCycConcepts() {

		return cycConcepts;
	}
	
	public void setCycConcepts(ArrayList<String> concepts) {
		cycConcepts = concepts;
	}

	public String getLabel() {

		return label;
	}

	public int getType() {

		return type;
	}

	public void setLabel( String label ) {

		this.label = label;
	}

	public void setType( int type ) {

		this.type = type;
	}

	public void setSynSetIDs( ArrayList<String> synsets ) {

		synSetIDs = synsets;
	}

	public String toString() {

		String str = "(" + label + "," + typeToString( type ) + ",[";

		str += synSetIDs.size() + "/";
		for ( Iterator<String> i = cycConcepts.iterator(); i.hasNext(); ) {
			str += i.next();
			if ( i.hasNext() )
				str += ",";
		}
		str += "])";

		return str;
	}
	


	public String typeToString(int type) {
		String str = "N/A";
		switch (type) {
		case TYPE_UNKNOWN:
			str = "?";
			break;
		case TYPE_NOUN:
			str = "N";
			break;
		case TYPE_VERB_INFINITIVE:
			str = "V";
			break;
		case TYPE_ADJ:
			str = "ADJ";
			break;
		case TYPE_ADV:
			str = "ADV";
			break;
		case TYPE_PARTICLE:
			str = "PART";
			break;
		case TYPE_NUMBER:
			str = "NUMBER";
			break;
		case TYPE_CONJ:
			str = "CONJ";
			break;
		case TYPE_PRONOUN:
			str = "PRON";
			break;
		case TYPE_PREPOSITION:
			str = "PP";
			break;
		case TYPE_ARTICLE:
			str = "DT";
			break;
		case TYPE_PUNCTUATION:
			str = ".";
			break;
		}
		return str;
	}

	/**
	 * Compares two Words for equality {@link Object.equals}
	 */
	public boolean equals( Object w ) {

		if ( ! ( w instanceof Word ) )
			return false;

		if ( label.equalsIgnoreCase( ( (Word) w ).getLabel() ) && ( (Word) w ).getType() == type )
			return true;
		return false;
	}
}
