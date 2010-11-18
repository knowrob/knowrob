package instruction.disambiguator;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import org.jdom.Document;
import org.jdom.Element;
import org.jdom.JDOMException;
import org.jdom.input.SAXBuilder;
import org.jdom.output.XMLOutputter;

public class DisambiguatorKB {

	// ==================================================================
	// XML tags and attributes
	// ==================================================================
	public static final String TAG_DISAMBIGUATOR = "disambiguator";
	public static final String TAG_PREPOSITIONS = "prepositions";
	public static final String TAG_INSTRUCTIONS = "instructions";
	public static final String TAG_PREPOSITION = "preposition";
	public static final String TAG_ACTION = "action";
	public static final String TAG_OBJECTS = "objects";
	public static final String TAG_OBJECT = "object";
	public static final String TAG_DEPENDENCY = "dependency";
	public static final String ATTR_COUNT = "count";
	public static final String ATTR_NAME = "name";
	public static final String ATTR_EVIDENCE = "evidence";

	private int instructionCount = 0;
	private int prepositionCount = 0;
	private int objectCount = 0;
	private String fileName = null;

	private HashMap<String, Preposition> prepositions = new HashMap<String, Preposition>();
	private HashMap<String, Action> actions = new HashMap<String, Action>();
	private HashMap<String, InstObject> objects = new HashMap<String, InstObject>();

	public DisambiguatorKB () {

	}

	/**
	 * Tells the Kb about the occurrence of a distinct preposition
	 * 
	 * @param p
	 */
	public void tellPreposition( String p ) {

		Preposition pp = prepositions.get( p );
		prepositionCount++;

		if ( pp == null ) {
			pp = new Preposition( p, 0 );
			prepositions.put( p, pp );
		}

		pp.count++;
	}

	/**
	 * Tells the KB about the object <code>object</code> given the action
	 * <code>evidence</code>
	 * 
	 * @param object
	 * @param evidence
	 */
	public void tellObject( String object, String evidence ) {

		InstObject o = objects.get( object );

		ArrayList<String> ev = new ArrayList<String>();
		ev.add( evidence );
		Condition c = new Condition( ev, 1 );

		objectCount++;

		if ( o != null ) {
			for ( int i = 0; i < o.conditions.size(); i++ ) {
				if ( o.conditions.get( i ).equals( c ) ) {
					o.conditions.get( i ).count++;
					return;
				}
			}
			o.conditions.add( c );
		}
		else {
			o = new InstObject( object );
			o.conditions.add( c );
			objects.put( object, o );
		}

	}

	/**
	 * Tells the KB about the occurence of the action <code>action</code> given
	 * the prepositions <code>evidence</code>
	 * 
	 * @param action
	 * @param evidence
	 */
	public void tellAction( String action, List<String> evidence ) {

		Action a = actions.get( action );

		Condition c = new Condition( evidence, 1 );

		instructionCount++;

		if ( a != null ) {
			a.count++;
			for ( int i = 0; i < a.conditions.size(); i++ ) {
				if ( a.conditions.get( i ).equals( c ) ) {
					a.conditions.get( i ).count++;
					return;
				}
			}
			a.conditions.add( c );
		}
		else {
			a = new Action( action, 1 );
			a.conditions.add( c );
			actions.put( action, a );
		}
	}

	/**
	 * Returns a probability-descending list of actions
	 * 
	 * @param evidence
	 * @return
	 */
	public List<String> askActions( List<String> evidence ) {

		ArrayList<String> sortedKeys = new ArrayList<String>( actions.keySet() );

		ProbabilityComparator comparator = new ProbabilityComparator( evidence, Action.class );

		Collections.sort( sortedKeys, comparator );

		ArrayList<String> sortedActions = new ArrayList<String>();
		for ( int i = 0; i < sortedKeys.size(); i++ )
			sortedActions.add( actions.get( sortedKeys.get( i ) ).name );

		return sortedActions;
	}

	/**
	 * Returns a probability-descending list of objects
	 * 
	 * @param evidence
	 * @return
	 */
	public List<String> askObjects( String evidence ) {

		ArrayList<String> sortedKeys = new ArrayList<String>( objects.keySet() );

		ArrayList<String> e = new ArrayList<String>();
		e.add( evidence );
		ProbabilityComparator comparator = new ProbabilityComparator( e, InstObject.class );

		Collections.sort( sortedKeys, comparator );
		ArrayList<String> sortedObjects = new ArrayList<String>();
		for ( int i = 0; i < sortedKeys.size(); i++ )
			sortedObjects.add( objects.get( sortedKeys.get( i ) ).name );

		return sortedObjects;
	}

	/**
	 * Returns a set of prepositions that are known in the KB
	 * 
	 * @return
	 */
	private List<Preposition> getPP() {

		ArrayList<Preposition> pp = new ArrayList<Preposition>();
		Set<String> keySet = prepositions.keySet();
		for ( Iterator<String> i = keySet.iterator(); i.hasNext(); )
			pp.add( prepositions.get( i.next() ) );

		return pp;
	}

	/**
	 * Computes the conditional probability P(action|evidence) of a distinct
	 * action given a set of prepositions as evidence
	 * 
	 * @param action
	 * @param evidence
	 * @return
	 */
	private double getProbabilityOfAction( String action, List<String> evidence ) {

		Action a = actions.get( action );
		Condition c = new Condition( evidence, - 1 );
		if ( a == null )
			return 0;

		int idx = a.conditions.indexOf( c );
		if ( idx < 0 )
			return 0;
		else {
			Condition cond = a.conditions.get( idx );

			// P(Action ^ Condition)
			double p_aAndp = ( (double) cond.count ) / ( (double) instructionCount );

			// P(on) * P(from) * ...
			double pp = 1.0;

			for ( int i = 0; i < getPP().size(); i++ ) {
				double p = ( (double) getPP().get( i ).count ) / ( (double) instructionCount );

				if ( cond.evidence.contains( getPP().get( i ).name ) )
					pp *= p;
				else
					continue;//pp *= ( 1.0 - p );
			}

			return p_aAndp / pp;
		}
	}

	/**
	 * Computes the conditional probability P(object|evidence), where evidence has
	 * to be an action
	 * 
	 * @param object
	 * @param evidence
	 * @return
	 */
	private double getProbabilityOfObject( String object, List<String> evidence ) {

		InstObject o = objects.get( object );
		Condition c = new Condition( evidence, - 1 );

		if ( o == null )
			return 0.;

		int idx = o.conditions.indexOf( c );
		if ( idx < 0 )
			return 0;
		else {
			Condition cond = o.conditions.get( idx );
			Action a = actions.get( evidence.get( 0 ) );
			double p_a = (double) a.count / (double) instructionCount;
			double p_oAnda = ( (double) cond.count / (double) instructionCount );

			return p_oAnda / p_a;
		}
	}

	/**
	 * Loads a stored KB snapshot from a file
	 * 
	 * @param fileName
	 * @throws IOException
	 * @throws JDOMException
	 */
	@SuppressWarnings("unchecked")
	public void load( String fileName ) throws JDOMException, IOException {

		// clear all
		prepositions.clear();
		actions.clear();
		objects.clear();

		this.fileName = fileName;

		File file = new File( fileName );

		if ( ! file.exists() )
			return;

		FileReader reader = new FileReader( file );

		SAXBuilder sax = new SAXBuilder();
		Document doc = sax.build( reader );

		Element disamb = doc.getRootElement();

		List<Element> elements = disamb.getChildren();

		for ( Iterator<Element> i = elements.iterator(); i.hasNext(); ) {
			Element el = i.next();

			// ==================================================================
			// Preposition-Count
			// ==================================================================
			if ( el.getName().equalsIgnoreCase( TAG_PREPOSITIONS ) )
				prepositionCount = Integer.valueOf( el.getAttributeValue( ATTR_COUNT ) );

			// ==================================================================
			// Instructions-Count
			// ==================================================================
			else if ( el.getName().equalsIgnoreCase( TAG_INSTRUCTIONS ) )
				instructionCount = Integer.valueOf( el.getAttributeValue( ATTR_COUNT ) );

			// ==================================================================
			// Read all elements
			// ==================================================================
			else if ( el.getName().equalsIgnoreCase( TAG_OBJECTS ) )
				objectCount = Integer.valueOf( el.getAttributeValue( ATTR_COUNT ) );

			// ==================================================================
			// Preposition
			// ==================================================================
			else if ( el.getName().equalsIgnoreCase( TAG_PREPOSITION ) ) {
				String name = el.getAttributeValue( ATTR_NAME );
				int count = Integer.valueOf( el.getAttributeValue( ATTR_COUNT ) );
				prepositions.put( name, new Preposition( name, count ) );
			}

			// ==================================================================
			// Action
			// ==================================================================
			else if ( el.getName().equalsIgnoreCase( TAG_ACTION ) ) {
				String name = el.getAttributeValue( ATTR_NAME );
				int count = Integer.valueOf( el.getAttributeValue( ATTR_COUNT ) );

				Action a = new Action( name, count );

				// ==================================================================
				// Dependencies
				// ==================================================================
				List<Element> cond = el.getChildren();
				for ( Iterator<Element> j = cond.iterator(); j.hasNext(); ) {
					Element e = j.next();
					String evidences = e.getAttributeValue( ATTR_EVIDENCE );
					int countEvid = Integer.valueOf( e.getAttributeValue( ATTR_COUNT ) );

					String[] evid = evidences.split( ";" );
					List<String> ev = new ArrayList<String>();
					for ( int k = 0; k < evid.length; k++ ) {
						if ( ! evid[k].isEmpty() )
							ev.add( evid[k] );
					}

					a.conditions.add( new Condition( ev, countEvid ) );
				}
				actions.put( name, a );
			}

			// ==================================================================
			// Object
			// ==================================================================
			else if ( el.getName().equalsIgnoreCase( TAG_OBJECT ) ) {

				String name = el.getAttributeValue( ATTR_NAME );

				InstObject o = new InstObject( name );

				// ==================================================================
				// Dependencies
				// ==================================================================
				List<Element> cond = el.getChildren();
				for ( Iterator<Element> j = cond.iterator(); j.hasNext(); ) {
					Element e = j.next();
					String evidences = e.getAttributeValue( ATTR_EVIDENCE );
					int countEvid = Integer.valueOf( e.getAttributeValue( ATTR_COUNT ) );

					String[] evid = evidences.split( ";" );
					List<String> ev = new ArrayList<String>();
					for ( int k = 0; k < evid.length; k++ )
						ev.add( evid[k] );

					o.conditions.add( new Condition( ev, countEvid ) );
				}
				objects.put( name, o );
			}
		}
	}

	/**
	 * Saves a snapshot of the KB in the file
	 * 
	 * @throws IOException
	 */
	public void save() throws IOException {

		if ( fileName != null )
			saveAs( fileName );
	}

	/**
	 * Saves a snapshot of the KB in a file
	 * 
	 * @param fileName
	 * @throws IOException
	 */
	public void saveAs( String fileName ) throws IOException {

		this.fileName = fileName;
		File file = new File( fileName );
		if ( ! file.exists() )
			file.createNewFile();

		Element disamb = new Element( TAG_DISAMBIGUATOR );

		// ==================================================================
		// Instruction-Count
		// ==================================================================
		Element instructions = new Element( TAG_INSTRUCTIONS );
		instructions.setAttribute( ATTR_COUNT, String.valueOf( instructionCount ) );
		disamb.addContent( instructions );

		// ==================================================================
		// Objects-Count
		// ==================================================================
		Element obj = new Element( TAG_OBJECTS );
		obj.setAttribute( ATTR_COUNT, String.valueOf( objectCount ) );
		disamb.addContent( obj );

		// ==================================================================
		// Preposition-Count
		// ==================================================================
		Element preposition_count = new Element( TAG_PREPOSITIONS );
		preposition_count.setAttribute( ATTR_COUNT, String.valueOf( prepositionCount ) );
		disamb.addContent( preposition_count );

		// ==================================================================
		// Prepositions
		// ==================================================================
		Set<String> keys = prepositions.keySet();
		for ( Iterator<String> i = keys.iterator(); i.hasNext(); ) {
			Preposition pp = prepositions.get( i.next() );

			Element p = new Element( TAG_PREPOSITION );
			p.setAttribute( ATTR_NAME, pp.name );
			p.setAttribute( ATTR_COUNT, String.valueOf( pp.count ) );
			disamb.addContent( p );
		}

		// ==================================================================
		// Actions
		// ==================================================================
		keys = actions.keySet();
		for ( Iterator<String> i = keys.iterator(); i.hasNext(); ) {
			Action a = actions.get( i.next() );
			Element e = new Element( TAG_ACTION );
			e.setAttribute( ATTR_NAME, a.name );
			e.setAttribute( ATTR_COUNT, String.valueOf( a.count ) );

			for ( int j = 0; j < a.conditions.size(); j++ ) {
				Element dependency = new Element( TAG_DEPENDENCY );
				dependency.setAttribute( ATTR_COUNT, String.valueOf( a.conditions.get( j ).count ) );
				dependency.setAttribute( ATTR_EVIDENCE, toArrayString( a.conditions.get( j ).evidence ) );
				e.addContent( dependency );
			}
			disamb.addContent( e );
		}

		// ==================================================================
		// Objects
		// ==================================================================
		keys = objects.keySet();
		for ( Iterator<String> i = keys.iterator(); i.hasNext(); ) {
			InstObject o = objects.get( i.next() );
			Element e = new Element( TAG_OBJECT );
			e.setAttribute( ATTR_NAME, o.name );

			for ( int j = 0; j < o.conditions.size(); j++ ) {
				Element dependency = new Element( TAG_DEPENDENCY );
				dependency.setAttribute( ATTR_COUNT, String.valueOf( o.conditions.get( j ).count ) );
				dependency.setAttribute( ATTR_EVIDENCE, toArrayString( o.conditions.get( j ).evidence ) );
				e.addContent( dependency );
			}
			disamb.addContent( e );
		}

		// ==================================================================
		// Create XML document and write it to file
		// ==================================================================
		Document doc = new Document( disamb );

		XMLOutputter outPutter = new XMLOutputter( " ", true );
		FileWriter outStream = new FileWriter( file );

		outPutter.output( doc, outStream );
	}

	public String toArrayString( List<String> list ) {

		String str = "";
		for ( int i = 0; i < list.size(); i++ ) {
			str += list.get( i );
			if ( i < list.size() - 1 )
				str += ";";
		}

		return str;
	}

	// ==================================================================
	// Inner classes for Knowledge representation
	// ==================================================================

	/**
	 * Represents a preposition and its occurance frequency
	 */
	private class Preposition {
		public String name;
		public int count;

		public Preposition ( String name, int count ) {

			this.name = name;
			this.count = count;
		}
	}

	/**
	 * Represents an evidence (dependency) for conditional probabilities
	 * 
	 * @author Daniel Nyga
	 * 
	 */
	private class Condition {
		public List<String> evidence;
		public int count;

		public Condition ( List<String> evidence, int count ) {

			this.evidence = evidence;
			this.count = count;
		}

		public boolean equals( Object o ) {

			if ( ! ( o instanceof Condition ) )
				return false;

			Condition c = (Condition) o;
			if ( c.evidence.isEmpty() && evidence.isEmpty() )
				return true;
			else if ( c.evidence.containsAll( evidence ) && evidence.containsAll( c.evidence ) )
				return true;
			else
				return false;
		}
	}

	/**
	 * Represents an action and its occurance frequency as well as its
	 * dependencies
	 * 
	 * @author Daniel Nyga
	 * 
	 */
	private class Action {
		public String name;
		public int count;
		public List<Condition> conditions = new ArrayList<Condition>();

		public Action ( String name, int count ) {

			this.name = name;
			this.count = count;
		}
	}

	/**
	 * Represents an object and its dependencies
	 * 
	 * @author Daniel Nyga
	 * 
	 */
	private class InstObject {
		public String name;
		public ArrayList<Condition> conditions = new ArrayList<Condition>();

		public InstObject ( String name ) {

			this.name = name;
		}
	}

	/**
	 * Comparator class for probabilistic comparison of actions and objects
	 * 
	 * @author Daniel Nyga
	 * 
	 */
	private class ProbabilityComparator implements Comparator<String> {

		List<String> evidence;

		Class type;

		public ProbabilityComparator ( List<String> evidence, Class c ) {

			this.evidence = evidence;
			this.type = c;
		}

		public int compare( String key1, String key2 ) {

			double p1, p2;

			if ( type == Action.class ) {
				Action a1 = actions.get( key1 );
				Action a2 = actions.get( key2 );

				p1 = getProbabilityOfAction( a1.name, evidence );
				p2 = getProbabilityOfAction( a2.name, evidence );
			}
			else if ( type == InstObject.class ) {
				InstObject o1 = objects.get( key1 );
				InstObject o2 = objects.get( key2 );

				p1 = getProbabilityOfObject( o1.name, evidence );
				p2 = getProbabilityOfObject( o2.name, evidence );
			}
			else
				return 0;

			if ( p1 < p2 )
				return 1;

			else if ( p1 > p2 )
				return - 1;

			return 0;
		}

	}
}
