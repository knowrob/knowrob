package instruction.disambiguator;

import java.io.IOException;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;
import org.opencyc.api.CycApiException;
import instruction.exceptions.AmbiguityException;
import instruction.exceptions.InstructionException;
import instruction.opencyc.OpenCyc;
import instruction.semanticObjects.Action;
import instruction.semanticObjects.Instruction;
import instruction.semanticObjects.ObjectX;
import instruction.semanticObjects.Preposition;
import instruction.semanticObjects.Quantifier;
import instruction.semanticObjects.SemanticObject;
import instruction.semanticObjects.Word;

public class Disambiguator {

	public static int MODE_APPLY = 0;
	public static int MODE_TRAIN = 1;

	private DisambiguatorKB kb;

	private int runMode;

	private IDisambiguationOracle oracle = null;
	
	public Disambiguator () {

		kb = new DisambiguatorKB();
	}
	
	public void setDisambiguationOracle(IDisambiguationOracle oracle) {
		this.oracle = oracle;
	}
	
	public IDisambiguationOracle getDisambiguationOracle() {
		return oracle;
	}

	public void setRunMode( int mode ) {

		runMode = mode;
		if ( runMode == MODE_APPLY )
			System.out.println( "Run Mode set to: APPLY-MODE" );

		else if ( runMode == MODE_TRAIN )
			System.out.println( "Run Mode set to: TRAINING-MODE" );
	}

	public void save( String fileName ) throws IOException {

		kb.saveAs( fileName );
	}

	public void load( String fileName ) throws Exception {

		kb.load( fileName );
		System.out.println( "Disambiguator KB snapshot loaded" );
	}

	public void disambiguateInstructions( List<Instruction> instructions ) throws AmbiguityException,
			InstructionException, UnknownHostException, CycApiException, IOException {

		for ( int i = 0; i < instructions.size(); i++ ) {
			ArrayList<String> prepositions = new ArrayList<String>();
			for ( int j = 0; j < instructions.get( i ).getPrepositions().size(); j++ ) {
				List<Preposition> pp = instructions.get( i ).getPrepositions();
				for ( int k = 0; k < pp.get( j ).getPrepositions().size(); k++ ) {
					if ( pp.get( j ).getPrepositions().get( k ).getType() == Word.TYPE_PREPOSITION ) {
						prepositions.add( pp.get( j ).getPrepositions().get( k ).getLabel() );
						if ( runMode == MODE_TRAIN )
							kb.tellPreposition( pp.get( j ).getPrepositions().get( k ).getLabel().toLowerCase() );
					}
				}
			}
			disambiguateInstruction( instructions.get( i ), instructions.get( i ).getAction().getAction()
					.getLabel(), prepositions, null );
		}
	}

	public void disambiguateInstruction( SemanticObject so, String action, List<String> prepositions, Instruction inst )
			throws UnknownHostException, IOException, CycApiException, AmbiguityException, InstructionException {

		// ==================================================================
		// Action
		// ==================================================================
		if ( so instanceof Action )
			disambiguateAction( (Action) so, prepositions, inst );

		// ==================================================================
		// Instruction
		// ==================================================================
		else if ( so instanceof Instruction ) {
			Instruction i = (Instruction) so;

			// Action
			disambiguateInstruction( i.getAction(), action, prepositions, i );
			action = i.getAction().getAction().getCycConcepts().get( 0 );

			// Objects
			List<ObjectX> objects = i.getObjects();
			for ( int j = 0; j < objects.size(); j++ )
				disambiguateInstruction( objects.get( j ), action, prepositions, i );

			// Prepositions
			List<Preposition> pp = i.getPrepositions();
			for ( int j = 0; j < pp.size(); j++ )
				disambiguateInstruction( pp.get( j ), action, prepositions, i );

			// Time constraint
			Quantifier q = i.getTimeConstraint();
			if ( q != null ) {
				for ( int j = 0; j < q.getAlternatives().size(); j++ ) {

				}
				if ( ! q.getMeasure().getLabel().isEmpty() ) {
					Word m = q.getMeasure();
					for ( int j = 0; j < m.getCycConcepts().size(); j++ ) {
						if ( OpenCyc.getInstance().isaInUnivVocMt( m.getCycConcepts().get( j ), "UnitOfTime" ) ) {
							String unitOfTime = m.getCycConcepts().get( j );
							m.getCycConcepts().clear();
							m.getCycConcepts().add( unitOfTime );
							break;
						}
					}
					// Remove time constraint if disambiguation failed
					if ( m.getCycConcepts().size() == 0 || m.getCycConcepts().size() > 1 )
						i.setTimeConstraint( null );
				}
			}
		}

		// ==================================================================
		// Object
		// ==================================================================
		else if ( so instanceof ObjectX ) {
			ObjectX o = (ObjectX) so;

			try {
				disambiguateObject( o, action, inst );
			}
			catch ( Exception e ) {
				System.out.println( "**********\n" + e.getMessage() );
			}

			List<Preposition> pp = o.getPrepositions();
			for ( int j = 0; j < pp.size(); j++ )
				disambiguateInstruction( pp.get( j ), action, prepositions, inst );
		}

		// ==================================================================
		// Preposition
		// ==================================================================
		else if ( so instanceof Preposition ) {
			Preposition pp = (Preposition) so;

			List<ObjectX> o = pp.getObjects();
			for ( int j = 0; j < o.size(); j++ )
				disambiguateInstruction( o.get( j ), action, prepositions, inst );
		}
	}

	/**
	 * Trys to disambiguate the Action by asking OpenCyc, whether there is a
	 * meaning of the action that can be executed by the Robot.
	 * 
	 * @param action
	 * @throws AmbiguityException
	 * @throws InstructionException
	 * @throws IOException
	 * @throws CycApiException
	 * @throws UnknownHostException
	 */
	public void disambiguateAction( Action action, List<String> prepositions, Instruction inst ) throws AmbiguityException,
			InstructionException, UnknownHostException, CycApiException, IOException {

		ArrayList<String> concepts = action.getAction().getCycConcepts();

		if ( runMode == MODE_APPLY ) {

			List<String> knownActions = kb.askActions( prepositions );

			for ( int i = 0; i < knownActions.size(); i++ ) {
				int idx = concepts.indexOf( knownActions.get( i ) );
				if ( idx < 0 )
					continue;
				else {
					concepts.clear();
					concepts.add( knownActions.get( i ) );
					break;
				}
			}

			String c = concepts.get( 0 );
			concepts.clear();
			concepts.add( c );
		}
		else if ( runMode == MODE_TRAIN ) {
			
			if (oracle == null)
				throw new AmbiguityException("No Disambiguation Oracle defined");
			
			String meaning = oracle.retrieveMeaningOfWord(action.getAction().getLabel(), concepts, inst);
			
			if (meaning == null || ! concepts.contains(meaning))
				meaning = concepts.get(0);

			kb.tellAction( meaning, prepositions );

		}
	}

	public void disambiguateObject( ObjectX o, String action, Instruction inst) throws InstructionException, AmbiguityException,
			UnknownHostException, CycApiException, IOException {

		for ( int k = 0; k < o.getName().size(); k++ ) {

			List<String> concepts = o.getName().get( k ).getCycConcepts();

			if ( runMode == MODE_APPLY ) {
				List<String> knownObjects = kb.askObjects( action );

				for ( int i = 0; i < knownObjects.size(); i++ ) {
					int idx = concepts.indexOf( knownObjects.get( i ) );
					if ( idx < 0 )
						continue;
					else {
						concepts.clear();
						concepts.add( knownObjects.get( i ) );
						break;
					}
				}

				String c = concepts.get( 0 );
				concepts.clear();
				concepts.add( c );
			}
			else if ( runMode == MODE_TRAIN ) {
				
				if (oracle == null)
					throw new AmbiguityException("No Disambiguation Oracle defined");
				
				String meaning = oracle.retrieveMeaningOfWord(o.getName().get(k).getLabel(), concepts, inst);
				
				if (meaning == null || ! concepts.contains(meaning))
					meaning = concepts.get(0);
				
				kb.tellObject( meaning, action );

			}

		}
	}
}
