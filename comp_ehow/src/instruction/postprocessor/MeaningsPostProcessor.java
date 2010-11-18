package instruction.postprocessor;

import instruction.configuration.ConfigurationManager;
import instruction.exceptions.CycMappingException;
import instruction.exceptions.WordNetException;
import instruction.importer.AddCycMappingListener;
import instruction.opencyc.OpenCyc;
import instruction.opencyc.OpenCyc20;
import instruction.semanticObjects.Action;
import instruction.semanticObjects.Instruction;
import instruction.semanticObjects.ObjectX;
import instruction.semanticObjects.Preposition;
import instruction.semanticObjects.Quantifier;
import instruction.semanticObjects.SemanticObject;
import instruction.semanticObjects.Word;
import instruction.wordnet.WordNetRDF2;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class MeaningsPostProcessor implements PostProcessor {
	
	AddCycMappingListener listener;

	public void run( List<Instruction> instructions ) {

		for ( int i = 0; i < instructions.size(); i++ ) {
			try {
				findMeanings( instructions.get( i ) );
			}
			catch ( Exception e ) {
				//e.printStackTrace();
				instructions.remove( i );
				i -= 1;
				continue;
			}
		}
	}

	/**
	 * Enriches the word <code>w</code> with Cyc Concepts that are mapped in
	 * <code>WordNetMappingMt</code><br>
	 * <b>NB:</b> The Synset-IDs must be stored via
	 * <code>Word.setSynsetIDs()</code> before.
	 * 
	 * @param w
	 * @throws Exception
	 */
	public void getCycConcepts( Word w ) throws Exception {

		ArrayList<String> synsetIDs = w.getSynSetIDs();

		w.getCycConcepts().clear();
 
		if ( synsetIDs.size() == 0 ) {
		//	System.out.println( "Word " + w.getLabel() + " is unknown in WordNet" );
			throw new WordNetException( w );
		}

		for ( Iterator<String> i = synsetIDs.iterator(); i.hasNext(); ) {
			String synset = i.next();

			ArrayList<String> concepts = OpenCyc.getInstance().getCycConceptFromWordNetID( synset );
			for ( Iterator<String> j = concepts.iterator(); j.hasNext(); ) {
				String c = j.next();
				if ( ! w.getCycConcepts().contains( c ) )
					w.getCycConcepts().add( c );
			}
		}

		if ( w.getCycConcepts().size() == 0 ) {
			if(listener != null) {
				String[] mapping = listener.getCycMappingForWord(w);
				if (mapping != null) {
					OpenCyc20.getInstance().addMapping(mapping[0], mapping[1]);
					List<String> old = ConfigurationManager.getMappings().get(mapping[1]);
					if(old == null)
						old = new ArrayList<String>();
					
					old.add(mapping[1]);
					w.getCycConcepts().add(mapping[1]);
					
					ConfigurationManager.getMappings().put(mapping[0], old);
					
					ConfigurationManager.saveSettings();
				}
			}
			else
				System.out.println( "Missing Cyc-Mapping for " + w.getLabel() );

			throw new CycMappingException(w);
		}
	}

	/**
	 * Retrieves the SynSet-IDs and corresponding Cyc Constants for the
	 * <code>SemanticObject</code> <code>so</code>
	 * 
	 * @param so
	 * @throws Exception
	 */
	public void findMeanings( SemanticObject so ) throws Exception {

		if ( so == null )
			return;

		// ==================================================================
		// Action
		// ==================================================================
		if ( so instanceof Action ) {
			Word a = ( (Action) so ).getAction();
			a.setSynSetIDs( WordNetRDF2.getSynsets( a.getLabel(), WordNetRDF2.convertPOS(a.getType()) ) );
			getCycConcepts( a );

			for ( int i = 0; i < ( (Action) so ).getAdverb().size(); i++ ) {
				try {
					Word adv = ( (Action) so ).getAdverb().get( i );
					adv.setSynSetIDs( WordNetRDF2.getSynsets( adv.getLabel(), WordNetRDF2.convertPOS(adv.getType()) ) );
					getCycConcepts( adv );
				}
				catch ( Exception e ) {
				}
			}
		}

		// ==================================================================
		// Instruction
		// ==================================================================
		else if ( so instanceof Instruction ) {
			Instruction i = (Instruction) so;

			// Action
			findMeanings( i.getAction() );

			// Objects
			List<ObjectX> objects = i.getObjects();
			for ( int j = 0; j < objects.size(); j++ ) {
				try {
					findMeanings( objects.get( j ) );
				}
				catch ( Exception e ) {
					objects.remove( j );
					j -= 1;
					continue;
				}
			}

			// Prepositions
			List<Preposition> pp = i.getPrepositions();
			for ( int j = 0; j < pp.size(); j++ ) {
				try {
					findMeanings( pp.get( j ) );
				}
				catch ( Exception e ) {
					pp.remove( j );
					j -= 1;
					continue;
				}
				if ( pp.get( j ).getObjects().size() == 0 ) {
					pp.remove( j );
					j -= 1;
					continue;
				}

			}
		}

		// ==================================================================
		// Object
		// ==================================================================
		else if ( so instanceof ObjectX ) {
			ObjectX o = (ObjectX) so;

			// Object Name
			for ( int j = 0; j < o.getName().size(); j++ ) {
				Word w = o.getName().get( j );
				w.setSynSetIDs( WordNetRDF2.getSynsets( w.getLabel(), WordNetRDF2.convertPOS(w.getType()) ) );
				getCycConcepts( w );
			}

			// Adjectives
			for ( int j = 0; j < o.getAdjectives().size(); j++ ) {
				Word w = o.getAdjectives().get( j );
				try {
					ArrayList<String> synset = WordNetRDF2.getSynsets( w.getLabel(), WordNetRDF2.convertPOS(w.getType()) );
					if (synset.isEmpty()) {
						synset = WordNetRDF2.getSynsets( w.getLabel(), WordNetRDF2.convertPOS(Word.TYPE_PAST_PARTICIPLE) );
					}
					w.setSynSetIDs( synset );
					getCycConcepts( w );
				}
				catch ( Exception e ) {
					o.getAdjectives().remove( j );
					j -= 1;
					continue;
				}
			}

			// Prepositions
			List<Preposition> pp = o.getPrepositions();
			for ( int j = 0; j < pp.size(); j++ ) {
				try {
					findMeanings( pp.get( j ) );
				}
				catch ( Exception e ) {
					pp.remove( j );
					j -= 1;
					continue;
				}
			}

			// Quantifier
			Quantifier q = o.getQuantifier();
			try {
				findMeanings( q );
			}
			catch ( Exception e ) {
				// o.setQuantifier( null );
			}
		}

		// ==================================================================
		// Prepositions
		// ==================================================================
		else if ( so instanceof Preposition ) {
			Preposition pp = (Preposition) so;
     
			// Objects
			List<ObjectX> o = pp.getObjects();
			for ( int j = 0; j < o.size(); j++ ) {
				try {
					findMeanings( o.get( j ) );
				}
				catch ( Exception e ) {
					o.remove( j );
					j -= 1;
					continue;
				}
			}
		}

		// ==================================================================
		// Quantifier
		// ==================================================================
		else if ( so instanceof Quantifier ) {
			Word m = ( (Quantifier) so ).getMeasure();
			if ( ! m.getLabel().isEmpty() ) {
				m.setSynSetIDs( WordNetRDF2.getSynsets( m.getLabel(), WordNetRDF2.convertPOS(m.getType())) );
				getCycConcepts( m );
			}
		}
	}
	
	public void setAddMappingListener(AddCycMappingListener listener) {
		this.listener = listener;
	}
}
