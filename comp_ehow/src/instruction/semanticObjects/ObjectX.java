package instruction.semanticObjects;

import java.util.ArrayList;
import java.util.List;

public class ObjectX extends SemanticObject {

	/**
	 * A list of Words that determine the object's name
	 */
	private List<Word> name = new ArrayList<Word>();

	/**
	 * Specifies an quantitative statement about the object
	 */
	private Quantifier quantifier = new Quantifier();

	/**
	 * Makes a prepositional statement about the object, e.g. its location
	 */
	private List<Preposition> prepositions = new ArrayList<Preposition>();

	/**
	 * Contains a set of adjectives that characterize the object more exactly
	 */
	private List<Word> adjectives = new ArrayList<Word>();

	/**
	 * Conatins a set of simple sentences that specify the object, e.g. relative
	 * clauses
	 */
	private List<Description> descriptions = new ArrayList<Description>();
	
	private List<Word> participles = new ArrayList<Word>();

	/**
	 * Constructs a new ObjectX object and adds the <code>Word</code> 
	 * <code>name</code>
	 * as a component to the object's name
	 * 
	 * @param obj
	 */
	public ObjectX ( Word name ) {

		this.name.add( name );
	}

	/**
	 * Constructs an empty <code>ObjectX</code>
	 */
	public ObjectX () {

	}

	public void addNameComponent( Word n ) {

		name.add( n );
	}

	public List<Word> getName() {

		return name;
	}

	public void setName( List<Word> n ) {

		name = n;
	}

	public Quantifier getQuantifier() {

		return quantifier;
	}

	public List<Description> getDescriptions() {

		return descriptions;
	}

	public void addDescription( Description desc ) {

		descriptions.add( desc );
	}

	public void setDescriptions( ArrayList<Description> descs ) {

		descriptions = descs;
	}

	public void setQuantifier( Quantifier q ) {

		quantifier = q;
	}

	public List<Word> getAdjectives() {

		return adjectives;
	}

	public void addPreposition( Preposition loc ) {

		prepositions.add( loc );
	}

	public List<Preposition> getPrepositions() {

		return prepositions;
	}

	public String toString() {

		String str = "(";

		for ( int i = 0; i < name.size(); i++ ) {
			str += name.get( i );
			if ( i < name.size() - 1 )
				str += ",";
		}
		str += ",";

		str += "{";
		for ( int i = 0; i < adjectives.size(); i++ ) {
			str += adjectives.get( i );
			if ( i + 1 < adjectives.size() )
				str += ",";
		}
		str += "},";

		str += "{";
		for ( int i = 0; i < prepositions.size(); i++ ) {
			str += prepositions.get( i );
			if ( i < prepositions.size() - 1 )
				str += ",";
		}
		str += "})";
		return str;
	}

	public boolean equals( Object o ) {

		if ( ! ( o instanceof ObjectX ) )
			return false;

		boolean equals = true;

		equals = equals && name.equals( ( (ObjectX) o ).getName() );
		equals = equals && adjectives.equals( ( (ObjectX) o ).getAdjectives() );
		equals = equals && prepositions.equals( ( (ObjectX) o ).getPrepositions() );
		equals = equals && quantifier.equals( ( (ObjectX) o ).getQuantifier() );

		return equals;
	}
	
	public void setParticiples( List<Word> participles ) {

		this.participles = participles;
	}
	
	public List<Word> getParticiples() {

		return participles;
	}
	
	public void addParticiple(Word part) {
		
		participles.add( part );
		
	}
}
