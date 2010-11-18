package instruction.semanticObjects;

import java.util.ArrayList;
import java.util.List;

public class Preposition extends SemanticObject {

	private List<Word> prepositions = new ArrayList<Word>();
	private List<ObjectX> objects = new ArrayList<ObjectX>();

	public Preposition () {

	}

	public Preposition ( Word p, ObjectX o ) {

		prepositions.add( p );
		objects.add( o );
	}
	
	public void setPrepositions(List<Word> newPP) {
		prepositions = newPP;
	}

	public List<ObjectX> getObjects() {

		return objects;
	}

	public List<Word> getPrepositions() {

		return prepositions;
	}

	public void addPreposition( Word pre ) {

		prepositions.add( pre );
	}

	public void addObject( ObjectX o ) {

		objects.add( o );
	}
	
	public void setObjects(List<ObjectX> newObjects) {
		
		objects = newObjects;
	}

	public String toString() {

		String str = "({";
		for ( int i = 0; i < prepositions.size(); i++ ) {
			str += prepositions.get( i ).getLabel();
			if ( i < prepositions.size() - 1 )
				str += ",";
		}
		str += "},{";
		for ( int i = 0; i < objects.size(); i++ ) {
			str += objects.get( i ).toString();
			if ( i < objects.size() - 1 )
				str += ",";
		}
		str += "})";
		return str;
	}

	public boolean equals( Object p ) {
		if (! (p instanceof Preposition))
			return false;
		boolean equals = true;

		equals = equals && prepositions.equals( ((Preposition) p).getPrepositions() );
		equals = equals && objects.equals( ((Preposition) p).getObjects() );
		return equals;
	}
}
