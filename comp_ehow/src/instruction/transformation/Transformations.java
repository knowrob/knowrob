package instruction.transformation;

import instruction.semanticObjects.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * This Class contains all ModelTransformation-Rules to transform given
 * SyntaxTrees into semantified {@link Instruction}-Objects
 * 
 * @author Daniel Nyga
 * 
 */
public class Transformations {

	private static HashMap<String, ModelTransformation> transformations = null;

	private static ModelTransformation defaultTransformation;

	/**
	 * Initializes all Transformations
	 */
	private static void init() {

		if ( transformations != null )
			return;

		/**
		 * The Map that holds the single Transformations
		 */
		transformations = new HashMap<String, ModelTransformation>();

		/**
		 * Transformation for Noun-Phrase Patterns
		 */
		transformations.put( "NP", new ModelTransformation() {

			public List<SemanticObject> doModelTransformation( List<SemanticObject> objects ) {

				List<SemanticObject> newObjects = new ArrayList<SemanticObject>();

				ObjectX obj = null;
				Quantifier q = null;

				for ( int i = 0; i < objects.size(); i++ ) {

					SemanticObject o = objects.get( i );

					if ( o instanceof Word ) {
						Word w = (Word) o;

						// Example Pattern: (NP (DT the)(NN pot))
						if ( w.getType() == Word.TYPE_NOUN ) {

							if ( obj == null ) {
								obj = new ObjectX();
								newObjects.add( obj );
							}

							obj.addNameComponent( w );
						}

						// Example Pattern: (NP (JJ cold)(NN water))
						else if ( w.getType() == Word.TYPE_ADJ ) {

							if ( obj == null ) {
								obj = new ObjectX();
								newObjects.add( obj );
							}
							obj.addNameComponent( w );
						}
						
					// Example Pattern: (NP (DT a)(JJ boiled)(NN egg))
						else if ( w.getType() == Word.TYPE_PAST_PARTICIPLE ) {

							if ( obj == null ) {
								obj = new ObjectX();
								newObjects.add( obj );
							}
							obj.getAdjectives().add( w );
						}

						// Example Pattern: (NP (PRP it))
						else if ( w.getType() == Word.TYPE_PRONOUN ) {
							if ( obj == null ) {
								obj = new ObjectX();
								newObjects.add( obj );
							}
							obj.addNameComponent( w );
						}

						// Example Pattern: (NP (CD 3)(NNS liters))
						else if ( w.getType() == Word.TYPE_NUMBER ) {
							// if ( obj == null ) {
							// obj = new ObjectX();
							// newObjects.add( obj );
							// }
							// obj.getQuantifier().getAlternatives().add( w );
							q = new Quantifier();
							q.getAlternatives().add( w );
						}

						// Example Pattern: (NP (DT a)(NN cup)(CC and)(NN
						// saucer))
						else if ( w.getType() == Word.TYPE_CONJ
								|| ( w.getType() == Word.TYPE_PUNCTUATION && w.getLabel().equals( "," ) ) ) {
							if ( obj != null ) {
								obj = null;
							}
						}
					}

					// Examle Pattern: (NP (NP (NN fish))(CC and)(NP (NNS
					// chips)))
					else if ( o instanceof ObjectX )
						newObjects.add( o );

					// Example Pattern: (NP (DT the)(NN cup)(PP (IN on)(NP (DT
					// the)(NN table))))
					else if ( o instanceof Preposition ) {
						for ( int j = 0; j < newObjects.size(); j++ ) {
							SemanticObject o2 = newObjects.get( j );
							if ( o2 instanceof ObjectX )
								( (ObjectX) o2 ).addPreposition( (Preposition) o );
						}
					}

					// Example Pattern: (NP (QP (CD 3)(TO to)(CD 4))(NNS eggs))
					else if ( o instanceof Quantifier ) {
						// if ( obj == null ) {
						// obj = new ObjectX();
						// newObjects.add( obj );
						// }
						// obj.getQuantifier().getAlternatives().addAll( ( (Quantifier) o
						// ).getAlternatives() );
						q = (Quantifier) o;
					}

					// Example: "...a pot that is large enough"
					else if ( o instanceof Description ) {
						for ( int j = 0; j < newObjects.size(); j++ ) {
							if ( newObjects.get( j ) instanceof ObjectX )
								( (ObjectX) newObjects.get( j ) ).addDescription( (Description) o );
						}
					}

					// Pass complete Instructions
					else if ( o instanceof Sentence )
						newObjects.add( o );

				}

				// add the quantifier to all objects
				for ( int i = 0; i < newObjects.size(); i++ ) {
					if ( newObjects.get( i ) instanceof ObjectX && q != null ) {
						( (ObjectX) newObjects.get( i ) ).setQuantifier( q );
					}
				}

				return newObjects;
			}
		} );

		/**
		 * Transformation for Verb-Phrase Patterns
		 */
		transformations.put( "VP", new ModelTransformation() {

			public List<SemanticObject> doModelTransformation( List<SemanticObject> obj ) {

				ArrayList<SemanticObject> newObj = new ArrayList<SemanticObject>();

				// Combine the upcoming SemanticObjects to a Description
				Sentence s = new Sentence();

				Action vp = new Action();
				ArrayList<ObjectX> np = new ArrayList<ObjectX>();
				ArrayList<Preposition> pp = new ArrayList<Preposition>();
				ArrayList<Precondition> pr = new ArrayList<Precondition>();
				ArrayList<Postcondition> po = new ArrayList<Postcondition>();

				// if an instruction has to be negated
				boolean negate = false;

				for ( int i = 0; i < obj.size(); i++ ) {

					SemanticObject o = obj.get( i );

					// Example Pattern: (VP (VB take)(NP (DT the)(NNS eggs))(CC
					// and)(NP (DT the)(NN milk)))
					if ( o instanceof ObjectX )
						np.add( (ObjectX) o );

					else if ( o instanceof Word ) {
						Word word = (Word) o;

						// Example Pattern: (VP (VB serve)(RB immediately))
						if ( word.getType() == Word.TYPE_ADV ) {

							// Example Pattern: (VP (VB do)(RB not)(VP (VB
							// stir))) (Negation)
							if ( word.getLabel().matches( "n't|not" ) )
								negate = true;
							else
								vp.getAdverb().add( word );
						}

						// Example Pattern: (VP (VB take)(PRT (RP out))(NP ...)
						else if ( word.getType() == Word.TYPE_PARTICLE )
							vp.getAction().setLabel( vp.getAction().getLabel() + " " + word.getLabel() );

						// Example Pattern: (VP (VB stir))
						else if ( word.getType() == Word.TYPE_VERB_INFINITIVE )
							vp.setAction( word );
					}

					// Gather ready objects to be added to the Description
					else if ( o instanceof Preposition )
						pp.add( (Preposition) o );

					else if ( o instanceof Precondition )
						pr.add( (Precondition) o );

					else if ( o instanceof Postcondition )
						po.add( (Postcondition) o );

					else if ( o instanceof Sentence ) {
						( (Instruction) o ).getAction().setDontDoIt( negate );
						newObj.add( o );
					}

					// else if ( o instanceof Instruction )
					// newObj.add( (Instruction) o );
				}

				// Add the gathered SemanticObjects to the Description
				s.setAction( vp );
				s.setObjects( np );
				s.setPreconditions( pr );
				s.setPostconditions( po );
				s.setPrepositions( pp );

				// Only let a Description pass if an Action is specified
				// and it's not an Action like "do" (for negations)
				if ( ! s.getAction().getAction().getLabel().isEmpty()
						&& ! s.getAction().getAction().getLabel().equalsIgnoreCase( "do" ) )
					newObj.add( s );
				return newObj;
			}

		} );

		/**
		 * Transformation for Sub-Sentence Patterns
		 */
		transformations.put( "SBAR", new ModelTransformation() {
			public List<SemanticObject> doModelTransformation( List<SemanticObject> objects ) {

				List<SemanticObject> newObjects = new ArrayList<SemanticObject>();

				int prepositionType = 0;

				for ( int i = 0; i < objects.size(); i++ ) {
					SemanticObject o = objects.get( i );

					// make out the type of the SBAR sub sentence
					if ( o instanceof Word ) {
						Word w = (Word) o;

						// Precondition
						if ( w.getType() == Word.TYPE_PREPOSITION && w.getLabel().toLowerCase().matches( "when|if|after" ) )
							prepositionType = 1;

						// Postcondition
						else if ( ( w.getType() == Word.TYPE_ADV || w.getType() == Word.TYPE_PREPOSITION )
								&& w.getLabel().toLowerCase().matches( "so|till|until" ) )
							prepositionType = 2;

						// Relative clause
						else if ( ( w.getType() == Word.TYPE_PRONOUN )
								&& w.getLabel().toLowerCase().matches( "that|which" ) )
							prepositionType = 3;

					}

					// SBAR starts with "if", "when", "after" or something like
					// that
					// => Precondition
					else if ( o instanceof Sentence && prepositionType == 1 ) {
						Precondition cond = new Precondition();
						cond.setCondition( (Sentence) o );
						newObjects.add( cond );
					}

					// SBAR starts with "so that" or something like that
					// => Postcondition
					else if ( o instanceof Sentence && prepositionType == 2 ) {
						Postcondition cond = new Postcondition();
						cond.setCondition( (Sentence) o );
						newObjects.add( cond );
					}

					// SBAR starts with "which" or "that" or something like that
					// => Relative clause
					else if ( o instanceof Sentence && prepositionType == 3 ) {
						Sentence s = (Sentence) o;
						Description d = new Description();

						d.setAction( s.getAction() );
						d.setObjects( s.getObjects() );
						d.setPostconditions( s.getPostconditions() );
						d.setPreconditions( s.getPreconditions() );
						d.setPrepositions( d.getPrepositions() );
						d.setActors( s.getActors() );

						newObjects.add( d );
					}

					// Re-Transform Instructions into Sentences, because Instructions
					// mustn't occour in SBARS; add Pre- and Postconditions
					else if ( o instanceof Instruction ) {
						Sentence s = new Sentence();
						s.setAction( ( (Instruction) o ).getAction() );
						s.setObjects( ( (Instruction) o ).getObjects() );
						s.setPostconditions( ( (Instruction) o ).getPostconditions() );
						s.setPreconditions( ( (Instruction) o ).getPreconditions() );
						s.setPrepositions( ( (Instruction) o ).getPrepositions() );

						if ( prepositionType == 1 ) {
							Precondition cond = new Precondition();
							cond.setCondition( s );
							newObjects.add( cond );
						}

						else if ( prepositionType == 2 ) {
							Postcondition cond = new Postcondition();
							cond.setCondition( s );
							newObjects.add( cond );
						}
						else if ( prepositionType == 3 ) {
							Instruction in = (Instruction) o;
							Description d = new Description();
							d.setAction( in.getAction() );
							d.setObjects( in.getObjects() );
							d.setPostconditions( in.getPostconditions() );
							d.setPreconditions( in.getPreconditions() );
							d.setPrepositions( in.getPrepositions() );
							newObjects.add( d );
						}
					}
				}

				return newObjects;
			}
		} );

		/**
		 * Transformation for Preposition-Phrase Patterns
		 */
		transformations.put( "PP", new ModelTransformation() {
			public List<SemanticObject> doModelTransformation( List<SemanticObject> objects ) {

				List<SemanticObject> newObjects = new ArrayList<SemanticObject>();
				Preposition p = null;

				for ( int i = 0; i < objects.size(); i++ ) {
					SemanticObject o = objects.get( i );

					if ( o instanceof ObjectX ) {
						if ( p == null )
							p = new Preposition();
						p.addObject( (ObjectX) o );
					}
					else if ( o instanceof Word ) {
						Word w = (Word) o;
						if ( w.getType() == Word.TYPE_PREPOSITION ) {
							if ( p == null )
								p = new Preposition();

							p.addPreposition( w );
						}
					}
					else if ( o instanceof Preposition )
						newObjects.add( o );
				}
				newObjects.add( p );
				return newObjects;
			}
		} );

		/**
		 * Transformation for Sentence-Patterns
		 */
		transformations.put( "S", new ModelTransformation() {
			public List<SemanticObject> doModelTransformation( List<SemanticObject> objects ) {

				List<SemanticObject> newObjects = new ArrayList<SemanticObject>();

				List<ObjectX> np = new ArrayList<ObjectX>();
				List<Preposition> pp = new ArrayList<Preposition>();
				List<Precondition> pr = new ArrayList<Precondition>();
				List<Postcondition> po = new ArrayList<Postcondition>();
				List<Word> adv = new ArrayList<Word>();

				int makeAllAlternative = Instruction.OPTIONAL_FALSE;

				for ( int i = 0; i < objects.size(); i++ ) {

					SemanticObject o = objects.get( i );

					// Example Pattern: (VP (VB take)(NP (DT the)(NNS eggs))(CC
					// and)(NP (DT the)(NN milk)))
					if ( o instanceof ObjectX )
						np.add( (ObjectX) o );

					else if ( o instanceof Preposition ) {
						pp.add( (Preposition) o );
					}
					else if ( o instanceof Precondition ) {
						pr.add( (Precondition) o );
					}
					else if ( o instanceof Postcondition ) {
						po.add( (Postcondition) o );
					}
					else if ( o instanceof Sentence ) {

						// If there have been already Nouns preceding the
						// verb, it's a Sentence, otherwise an Instruction
						if ( ! np.isEmpty() ) {
							for ( int k = 0; k < np.size(); k++ ) {
								( (Sentence) o ).addActor( np.get( k ) );
								np.remove( k );
							}
							for ( int k = 0; k < pp.size(); k++ ) {
								( (Sentence) o ).addPreposition( pp.get( k ) );
								pp.remove( k );
							}
							newObjects.add( o );
						}
						else {
							// Transform the Sentence into an Instruction
							Sentence d = (Sentence) o;
							Instruction inst = new Instruction();
							inst.setAction( d.getAction() );
							inst.setObjects( d.getObjects() );
							inst.setPreconditions( d.getPreconditions() );
							inst.setPostconditions( d.getPostconditions() );
							inst.setPrepositions( d.getPrepositions() );

							newObjects.add( inst );
						}
					}

					else if ( o instanceof Description ) {
						Description d = (Description) o;
						Instruction inst = new Instruction();
						inst.setAction( d.getAction() );
						inst.setObjects( d.getObjects() );
						inst.setPreconditions( d.getPreconditions() );
						inst.setPostconditions( d.getPostconditions() );
						inst.setPrepositions( d.getPrepositions() );

						newObjects.add( inst );
					}

					else if ( o instanceof Instruction )
						newObjects.add( o );

					else if ( o instanceof Word ) {
						Word w = (Word) o;
						if ( w.getType() == Word.TYPE_CONJ && w.getLabel().equalsIgnoreCase( "or" ) ) {
							if ( np.isEmpty() && pp.isEmpty() && pr.isEmpty() && po.isEmpty() )
								makeAllAlternative = Instruction.OPTIONAL_GLOBAL;
							else
								makeAllAlternative = Instruction.OPTIONAL_LOCAL;
						}
						else if ( w.getType() == Word.TYPE_ADV )
							adv.add( w );
					}
				}

				for ( int i = 0; i < newObjects.size(); i++ ) {
					if ( newObjects.get( i ) instanceof Instruction && ! pr.isEmpty() ) {
						for ( int j = 0; j < pr.size(); j++ )
							( (Instruction) newObjects.get( i ) ).getPreconditions().add( pr.get( j ) );
					}
					if ( newObjects.get( i ) instanceof Instruction ) {
						Instruction in = (Instruction) newObjects.get( i );
						in.setOptional( makeAllAlternative );
						in.getAction().getAdverb().addAll( adv );
					}
				}

				return newObjects;
			}
		} );

		/**
		 * Model Transformation for Quantifier Patterns
		 */
		transformations.put( "QP", new ModelTransformation() {
			public List<SemanticObject> doModelTransformation( List<SemanticObject> objects ) {

				List<SemanticObject> newObjects = new ArrayList<SemanticObject>();
				Quantifier q = new Quantifier();

				// Example Pattern: (QP (CD 1)(CC or)(CD 2))
				for ( int i = 0; i < objects.size(); i++ ) {
					SemanticObject o = objects.get( i );
					if ( o instanceof Word ) {
						Word w = (Word) o;
						if ( w.getType() == Word.TYPE_NUMBER ) {
							q.getAlternatives().add( w );
						}
					}
				}

				newObjects.add( q );

				return newObjects;
			}
		} );

		/**
		 * Model Transformation for Adverb Phrase Patterns
		 */
		transformations.put( "ADVP", new ModelTransformation() {

			public List<SemanticObject> doModelTransformation( List<SemanticObject> objects ) {

				List<SemanticObject> newObjects = new ArrayList<SemanticObject>();

				// Check, if the ADVP contains a Preposition.
				// If so, it is likely that this ADVP is malformed and the Words
				// before the Preposition Object are Prepositions

				boolean foundPP = false;

				// Example Pattern: (ADVP (RB right)(JJ next)(PP (TO to)(NP
				// ...))) (malformed pattern)
				for ( int i = 0; i < objects.size(); i++ ) {
					SemanticObject o = objects.get( i );
					if ( o instanceof Preposition ) {
						foundPP = true;
						Preposition p = (Preposition) o;
						for ( int j = i - 1; j >= 0; j-- ) {
							SemanticObject o2 = objects.get( j );
							if ( o2 instanceof Word ) {
								p.getPrepositions().add( 0, (Word) o2 );
								newObjects.remove( o2 );
							}
							else
								break;
						}
						newObjects.add( p );
					}
					else
						newObjects.add( o );
				}

				if ( ! foundPP )
					return defaultTransformation.doModelTransformation( objects );

				return newObjects;
			}
		} );

		/**
		 * Model Transformation for PRN-Phrases (Brackets)
		 */
		transformations.put( "PRN", new ModelTransformation() {

			public List<SemanticObject> doModelTransformation( List<SemanticObject> objects ) {

				List<SemanticObject> newObjects = new ArrayList<SemanticObject>();

				// For brackets: only let complete instructions pass
				for ( int i = 0; i < objects.size(); i++ ) {
					SemanticObject o = objects.get( i );
					if ( o instanceof Instruction )
						newObjects.add( o );
				}

				return newObjects;
			}
		} );

		/**
		 * Model Transformation for ROOT-Phrases Filter for inconsistent
		 * Instructions
		 */
		transformations.put( "ROOT", new ModelTransformation() {

			public List<SemanticObject> doModelTransformation( List<SemanticObject> objects ) {

				List<SemanticObject> newObjects = new ArrayList<SemanticObject>();

				// For brackets: only let complete instructions pass
				for ( int i = 0; i < objects.size(); i++ ) {
					SemanticObject o = objects.get( i );
					if ( o != null && o instanceof Instruction && ! ( o instanceof Description )
							&& ! ( o instanceof Sentence ) ) {
						//Instruction inst = (Instruction) o;
						// if (WordNet.wordExactlyExistsAs(
						// inst.getAction().getAction().getLabel(), Word.TYPE_VERB ))
						newObjects.add( o );
					}
				}

				return newObjects;
			}
		} );

		/**
		 * Model Transformation for ROOT-Phrases Filter for inconsistent
		 * Instructions
		 */
		transformations.put( "ADJP", new ModelTransformation() {
			public List<SemanticObject> doModelTransformation( List<SemanticObject> objects ) {

				List<SemanticObject> newObjects = new ArrayList<SemanticObject>();

				Quantifier q = null;

				// For brackets: only let complete instructions pass
				for ( int i = 0; i < objects.size(); i++ ) {
					SemanticObject o = objects.get( i );
					if ( o instanceof Word ) {
						Word w = (Word) o;
						if ( w.getType() == Word.TYPE_NUMBER ) {
							q = new Quantifier();
							q.getAlternatives().add( w );
						}
						else if ( q != null ) {
							q.setMeasure( w );
						}
					}
				}

				newObjects.add( q );
				return newObjects;
			}
		} );

		/**
		 * Model Transformation for FRAG-Phrases Filter for inconsistent
		 * Instructions
		 */
		transformations.put( "FRAG", get( "S" ) );

		/**
		 * Does nothing but directing the input SemanticObjects
		 */
		defaultTransformation = new ModelTransformation() {

			public List<SemanticObject> doModelTransformation( List<SemanticObject> objects ) {

				return objects;
			}

		};
	}

	/**
	 * Returns the ModelTransformation Object that fits to type SyntaxTree Node,
	 * or the DefaultTransformation, if no Transformation is there.
	 * 
	 * @param key
	 * @return
	 */
	public static ModelTransformation get( String key ) {

		if ( transformations == null )
			init();
		ModelTransformation ret = transformations.get( key );
		if ( ret == null )
			return defaultTransformation;
		return ret;
	}

}
