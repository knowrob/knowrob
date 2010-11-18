package instruction.converter;

import instruction.opencyc.OpenCyc;
import instruction.semanticObjects.Action;
import instruction.semanticObjects.Instruction;
import instruction.semanticObjects.ObjectX;
import instruction.semanticObjects.Preposition;
import instruction.semanticObjects.Quantifier;
import instruction.semanticObjects.Word;
import instruction.wordnet.PrepositionalMap;
import java.io.IOException;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.opencyc.api.CycApiException;

public class Instruction2CycLConverter {

	/** List of all assertions that have been made */
	List<String> assertions = new ArrayList<String>();
	String planName = "";

	/**
	 * Cache for all Objects intantiated during the howto. Maps Concepts to
	 * instance names
	 */
	private HashMap<String, String> objectInstances = new HashMap<String, String>();

	/**
	 * Creates a new instance of the given concept in Cyc. The instance is named
	 * by the concept name in lower case letters + the smallest non-zero integer
	 * value, which has not yet been used. The type of the instance is only valid
	 * in the <code>TUMKitchenEnvironmentMt</code> Example: <code>napkin1</code>,
	 * <code>napkin2</code>, <code>napkin3</code> - when
	 * <code>makeInstanceOfObject("Napkin")</code> was called 3 times.
	 * 
	 * @param concept
	 * @return The Name of the instance
	 * @throws CycApiException
	 * @throws UnknownHostException
	 * @throws IOException
	 */
	public String makeInstanceOfObject( String concept ) throws CycApiException, UnknownHostException,
			IOException {

		if ( concept == null || concept.isEmpty() )
			return null;

		if ( objectInstances.get( concept ) != null )
			return objectInstances.get( concept );

		// Find a fitting instance name
		int counter = 1;
		String instName;
		do {
			instName = concept.toLowerCase() + counter++;
		}
		while ( OpenCyc.getInstance().constantExists( instName ) );

		// create the assertion
		String assertion = "(#$isa #$" + instName + " #$" + concept + ")";

		// make constant
		OpenCyc.getInstance().getCycAcces().makeCycConstant( instName );

		// make assertion
		OpenCyc.getInstance().makeAssertionInKitchenMt( assertion );
		assertions.add( assertion );

		objectInstances.put( concept, instName );

		return instName;
	}

	/**
	 * Creates a new instance of an Action given by the Concept
	 * <code>concept</code>.The instance is named by the concept name in lower
	 * case letters + the smallest non-zero integer value, which has not yet been
	 * used, if <code>nameProposal</code> is <code>null</code><br/>
	 * 
	 * @param concept
	 * @param nameProposal
	 * @return
	 * @throws CycApiException
	 * @throws UnknownHostException
	 * @throws IOException
	 */
	public String makeInstanceOfAction( String concept, String nameProposal ) throws CycApiException,
			UnknownHostException, IOException {

		if ( concept == null || concept.isEmpty() )
			return null;

		// Find a fitting instance name
		int counter = 1;
		String instName;
		do {
			instName = ( nameProposal == null ? concept.toLowerCase() : nameProposal ) + counter++;
		}
		while ( OpenCyc.getInstance().constantExists( instName ) );

		// create the assertion
		String assertion = "(#$isa #$" + instName + " #$" + concept + ")";

		// make constant
		OpenCyc.getInstance().getCycAcces().makeCycConstant( instName );

		// make assertion
		OpenCyc.getInstance().makeAssertionInKitchenMt( assertion );
		assertions.add( assertion );

		return instName;
	}

	/**
	 * Converts a single <code>Instruction</code> into CycL and saves it in the
	 * Cyc KB.
	 * 
	 * @param inst
	 * @return
	 * @throws CycApiException
	 * @throws UnknownHostException
	 * @throws IOException
	 */
	public String convertInstruction( Instruction inst ) throws CycApiException, UnknownHostException,
			IOException {

		try {
			// ==================================================================
			// Convert the Action
			// ==================================================================
			Word action = inst.getAction().getAction();
			String actionInst = makeInstanceOfAction( action.getCycConcepts().get( 0 ), null );

			// ==================================================================
			// Convert all Objects
			// ==================================================================
			List<ObjectX> objects = inst.getObjects();
			List<String> objectInstances = convertObjects( objects, actionInst );

			for ( int i = 0; i < objectInstances.size(); i++ ) {

				// add the "objectActedOn" predicate to the object
				String assertion = "(#$objectActedOn #$" + actionInst + " #$" + objectInstances.get( i ) + ")";
				OpenCyc.getInstance().makeAssertionInKitchenMt( assertion );
				assertions.add( assertion );
			}

			// ==================================================================
			// Convert all Prepositions
			// ==================================================================
			for ( int i = 0; i < inst.getPrepositions().size(); i++ ) {
				Preposition pp = inst.getPrepositions().get( i );
				convertPP( pp, actionInst, objectInstances );
			}

			// ==================================================================
			// Convert time constraint
			// ==================================================================
			if ( inst.getTimeConstraint() != null ) {
				convertTimeConstraint( inst.getTimeConstraint(), actionInst );
			}

			// ==================================================================
			// Make the SimpleActionPrediacte in TUMKitchenEnvironmentMt
			// ==================================================================
			String assertion = "(#$isa #$" + actionInst + " #$SimpleActionPredicate)";
			OpenCyc.getInstance().makeAssertionInKitchenPlanningMt( assertion );
			assertions.add( assertion );

			return actionInst;
		}
		catch ( Exception e ) {
			e.printStackTrace();
			return null;
		}
	}

	/**
	 * 
	 * @param tc
	 * @param actionInst
	 */
	public void convertTimeConstraint( Quantifier tc, String actionInst ) {

		// ==================================================================
		// Make out a maximum range of quantifying figures taking the first
		// and last one of the Quantifier, or an underspecified Quantity
		// ==================================================================
		List<String> range = new ArrayList<String>();

		for ( int i = 0; i < tc.getAlternatives().size(); i++ ) {
			Word w = tc.getAlternatives().get( i );

			// Check for a quotient
			if ( w.getLabel().split( "\\\\/" ).length > 1 ) {
				String[] tokens = w.getLabel().split( "\\\\/" );
				range.add( "(#$QuotientFn " + tokens[0] + " " + tokens[1] + ")" );
				continue;
			}

			// Check for underspecified amount
			if ( w.getCycConcepts().size() > 0 ) {
				range.add( "#$" + w.getCycConcepts().get( 0 ) );
				continue;
			}

			// Check for figures
			try {
				Double.valueOf( w.getLabel() );
				range.add( w.getLabel() );
			}
			catch ( Exception e ) {
				e.printStackTrace();
			}
		}

		if ( range.isEmpty() )
			return;

		String measure;

		if ( tc.getMeasure().getCycConcepts().size() > 0 )
			measure = tc.getMeasure().getCycConcepts().get( 0 );
		else
			return;

		String assertion = "(#$timeSpan #$" + actionInst + " (#$" + measure + " " + range.get( 0 )
				+ ( range.size() > 1 ? ( " " + range.get( range.size() - 1 ) ) : "" ) + "))";

		try {
			OpenCyc.getInstance().makeAssertionInKitchenMt( assertion );
			assertions.add( assertion );
		}
		catch ( Exception e ) {
			e.printStackTrace();
		}
	}

	/**
	 * Converts a whole HowTo into CycL and saves it in the Cyc KB.
	 * 
	 * @param instructionList
	 *          List of Instructions of the HowTo
	 * @param howto
	 *          <code>Instruction</code> representing the "name" of the HowTo
	 * @throws CycApiException
	 * @throws UnknownHostException
	 * @throws IOException
	 */
	public void convertHowTo( List<Instruction> instructionList, Instruction howto ) throws CycApiException,
			UnknownHostException, IOException {

		resetConverter();
		
		howto = new Instruction();
		Word set = new Word(Word.TYPE_VERB_INFINITIVE, "make");
		ArrayList<String> list = new ArrayList<String>();
		list.add("MakingSomething");
		set.setCycConcepts(list);
		
		Word table = new Word(Word.TYPE_NOUN, "pancakes");
		ArrayList<String> list2 = new ArrayList<String>();
		list2.add("Pancake");
		table.setCycConcepts(list2);
		
		howto.setAction(new Action(set));
		howto.addObject(new ObjectX(table));

		// ==================================================================
		// Convert the Action and Object defining the HowTo
		// ==================================================================
		String howtoInst = makeInstanceOfAction( howto.getAction().getAction().getCycConcepts().get( 0 ), howto
				.getAction().getAction().getLabel()
				+ "_" + howto.getObjects().get( 0 ).getName().get( 0 ).getLabel().replaceAll( " ", "_" ) );
		
		String howtoObjInst = convertObjects( howto.getObjects(), howtoInst ).get( 0 );

		String assertion = "(#$objectActedOn #$" + howtoInst + " #$" + howtoObjInst + ")";
		OpenCyc.getInstance().makeAssertionInKitchenMt( assertion );
		assertions.add( assertion );

		// ==================================================================
		// Convert every single Instruction
		// ==================================================================
		Map<String, Instruction> actionInstancesMap = new HashMap<String, Instruction>();
		ArrayList<String> actionInstances = new ArrayList<String>();
		for ( int i = 0; i < instructionList.size(); i++ ) {
			try {
				String actIn = convertInstruction( instructionList.get( i ) );
				if ( actIn != null ) {
					actionInstancesMap.put( actIn, instructionList.get( i ) );
					actionInstances.add( actIn );
				}
			}
			catch ( Exception e ) {
				e.printStackTrace();
			}
		}

		// ==================================================================
		// Create an action sequence as a plan
		// ==================================================================
		String actionSequence = "";
		boolean optionalSequence = false;
		for ( int i = 0; i < actionInstances.size(); i++ ) {
			if ( actionInstancesMap.get( actionInstances.get( i ) ).getAction().dontDoIt() )
				actionSequence += "(#$notDo #$" + actionInstances.get( i ) + ")";
			else if ( actionInstancesMap.get( actionInstances.get( i ) ).isOptional() && ! optionalSequence ) {
				actionSequence += "(#$orDo #$" + actionInstances.get( i );
				optionalSequence = true;
			}
			else {
				if ( optionalSequence && ! actionInstancesMap.get( actionInstances.get( i ) ).isOptional() ) {
					actionSequence += ") ";
					optionalSequence = false;
				}
				actionSequence += "#$" + actionInstances.get( i );
			}

			if ( i < actionInstances.size() - 1 )
				actionSequence += " ";
		}

		assertion = "(#$isa #$" + howtoInst + " #$ActionPredicate" + ")";
		OpenCyc.getInstance().makeAssertionInKitchenPlanningMt( assertion );
		assertions.add( assertion );

		assertion = "(#$arity #$" + howtoInst + " 1)";
		OpenCyc.getInstance().makeAssertionInKitchenPlanningMt( assertion );
		assertions.add( assertion );

		assertion = "(#$arg1Isa #$" + howtoInst + " #$"
				+ howto.getObjects().get( 0 ).getName().get( 0 ).getCycConcepts().get( 0 ) + ")";
		OpenCyc.getInstance().makeAssertionInKitchenPlanningMt( assertion );
		assertions.add( assertion );

		assertion = "(#$isa #$" + howtoObjInst + " #$"
				+ howto.getObjects().get( 0 ).getName().get( 0 ).getCycConcepts().get( 0 ) + ")";
		OpenCyc.getInstance().makeAssertionInKitchenPlanningMt( assertion );
		assertions.add( assertion );

		assertion = "(#$methodForAction (#$" + howtoInst + " #$" + howtoObjInst
				+ ") (#$actionSequence (#$TheList " + actionSequence + ")))";
		OpenCyc.getInstance().makeAssertionInKitchenPlanningMt( assertion );
		planName = "(#$" + howtoInst + " #$" + howtoObjInst + ")";
		assertions.add( assertion );
	}

	/**
	 * Converts a Preposition an root level, i.e. directly stored in an
	 * Instruction into CycL and saves it in the Cyc KB
	 * 
	 * @param pp
	 * @param actionInstance
	 * @param parentObjects
	 * @throws CycApiException
	 * @throws UnknownHostException
	 * @throws IOException
	 */
	public void convertPP( Preposition pp, String actionInstance, List<String> parentObjects )
			throws CycApiException, UnknownHostException, IOException {

		List<String> objectInstances = null;

		// ==================================================================
		// Instantiate all objects of this Preposition
		// ==================================================================
		objectInstances = convertObjects( pp.getObjects(), actionInstance );

		// ==================================================================
		// Connect each parent Object with each child Object for each preposition
		// ==================================================================
		for ( int i = 0; i < objectInstances.size(); i++ ) {
			String child = objectInstances.get( i );

			for ( int k = 0; k < parentObjects.size(); k++ ) {
				String parent = parentObjects.get( k );

				for ( int j = 0; j < pp.getPrepositions().size(); j++ )
					connectObjectWithPreposition( child, parent, pp.getPrepositions().get( j ), actionInstance );

			}

			// if there are no objects specified in the instruction, connect the
			// action
			// instance with the prepositional objects
			if ( parentObjects.size() == 0 ) {
				for ( int j = 0; j < pp.getPrepositions().size(); j++ )
					connectObjectWithPreposition( child, actionInstance, pp.getPrepositions().get( j ), actionInstance );
			}
		}
	}

	/**
	 * Converts the prepositional relation between two Objects <code>child</code>
	 * and <code>parent</code> into CycL and saves it in the Cyc KB.
	 * 
	 * @param child
	 *          instance name of the child object
	 * @param parent
	 *          instance name of the parent object
	 * @param pp
	 *          the preposition connecting <code>child</code> and
	 *          <code>parent</code>
	 * @param action
	 */
	public void connectObjectWithPreposition( String child, String parent, Word pp, String action ) {

		if ( pp.getType() != Word.TYPE_PREPOSITION )
			return;

		String p = PrepositionalMap.get( pp.getLabel() );

		if ( p == null )
			return;

		// ==================================================================
		// Generate the assertion
		// ==================================================================
		String assertion;

		if ( p.equals( PrepositionalMap.ON ) ) {
			assertion = "(#$" + PrepositionalMap.ON + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.AT ) ) {
			assertion = "(#$" + PrepositionalMap.AT + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.FROM ) ) {
			assertion = "(#$" + PrepositionalMap.FROM + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.IN ) ) {
			assertion = "(#$" + PrepositionalMap.IN + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.IN_FRONT_OF ) ) {
			assertion = "(#$" + PrepositionalMap.IN_FRONT_OF + " #$" + parent + " " + child + ")";
		}
		else if ( p.equals( PrepositionalMap.NEXT_TO ) ) {
			assertion = "(#$" + PrepositionalMap.NEXT_TO + " #$" + parent + " #$" + child + " #$SomethingExisting)";
		}
		else if ( p.equals( PrepositionalMap.OF ) ) {
			assertion = "(#$" + PrepositionalMap.OF + " #$" + child + " #$" + parent + ")";
		}
		else if ( p.equals( PrepositionalMap.TO ) ) {
			assertion = "(#$" + PrepositionalMap.TO + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.FOR ) ) {
			assertion = "(#$" + PrepositionalMap.FOR + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.THROUGH ) ) {
			assertion = "(#$" + PrepositionalMap.THROUGH + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.AMONG ) ) {
			assertion = "(#$" + PrepositionalMap.AMONG + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.INSIDE ) ) {
			assertion = "(#$" + PrepositionalMap.INSIDE + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.ABOUT ) ) {
			assertion = "(#$" + PrepositionalMap.ABOUT + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.WITHOUT ) ) {
			assertion = "(#$" + PrepositionalMap.WITHOUT + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.ALONG ) ) {
			assertion = "(#$" + PrepositionalMap.ALONG + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.AROUND ) ) {
			assertion = "(#$" + PrepositionalMap.AROUND + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.BY ) ) {
			assertion = "(#$" + PrepositionalMap.BY + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.UNDER ) ) {
			assertion = "(#$" + PrepositionalMap.UNDER + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.OVER ) ) {
			assertion = "(#$" + PrepositionalMap.OVER + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.WITH ) ) {
			assertion = "(#$" + PrepositionalMap.WITH + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.INTO ) ) {
			assertion = "(#$" + PrepositionalMap.INTO + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.ACROSS ) ) {
			assertion = "(#$" + PrepositionalMap.ACROSS + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.FOR ) ) {
			assertion = "(#$" + PrepositionalMap.FOR + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.AFTER ) ) {
			assertion = "(#$" + PrepositionalMap.AFTER + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.BEFORE ) ) {
			assertion = "(#$" + PrepositionalMap.BEFORE + " #$" + parent + " #$" + child + ")";
		}
		else if ( p.equals( PrepositionalMap.AGAINST ) ) {
			assertion = "(#$" + PrepositionalMap.AGAINST + " #$" + parent + " #$" + child + ")";
		}
		else
			return;

		if ( action != null )
			assertion = "(#$purposeOf-Generic #$" + action + " " + assertion + ")";

		// ==================================================================
		// Make the assertion
		// ==================================================================
		try {
			OpenCyc.getInstance().makeAssertionInKitchenMt( assertion );
			assertions.add( assertion );
		}
		catch ( Exception e ) {
			e.printStackTrace();
		}
	}

	/**
	 * Converts a set of <code>ObjectX</code>s into CycL and saves it in the Cyc
	 * KB
	 * 
	 * @param objects
	 * @return
	 * @throws CycApiException
	 * @throws UnknownHostException
	 * @throws IOException
	 */
	public List<String> convertObjects( List<ObjectX> objects, String actionInstance ) throws CycApiException, UnknownHostException,
			IOException {

		List<String> objectInstances = new ArrayList<String>();

		// ==================================================================
		// Create instance of object
		// ==================================================================
		for ( int i = 0; i < objects.size(); i++ ) {
			if ( objects.get( i ).getName().size() == 0 )
				continue;
			String instance = makeInstanceOfObject( objects.get( i ).getName().get( 0 ).getCycConcepts().get( 0 ) );
			if ( instance != null && ! instance.isEmpty() ) {
				objectInstances.add( instance );

				// ==================================================================
				// Convert Adjectives of object
				// ==================================================================
				convertAdjectives( objects.get( i ).getAdjectives(), instance, actionInstance );

				// ==================================================================
				// Convert Quantifier of object
				// ==================================================================
				convertQuantifier( objects.get( i ).getQuantifier(), instance );
			}
		}

		// ==================================================================
		// Convert all Prepositions of the objects
		// ==================================================================
		for ( int i = 0; i < objects.size(); i++ ) {
			ObjectX o = objects.get( i );

			for ( int j = 0; j < o.getPrepositions().size(); j++ )
				convertNestedPP( o.getPrepositions().get( j ), objectInstances, actionInstance );
		}

		return objectInstances;
	}

	/**
	 * Converts a list of adjectives of an object that has been instantiated with
	 * the name <code>objectInstance</code>
	 * 
	 * @param adjectives
	 * @param objectInstance
	 */
	public void convertAdjectives( List<Word> adjectives, String objectInstance, String actionInstance ) {

		for ( int i = 0; i < adjectives.size(); i++ ) {

			List<String> concepts = adjectives.get( i ).getCycConcepts();
			try {
				String assertion = null;

				// ==================================================================
				// 1. Case: Only 1 Concept generalized by SomethingExisting
				// ==================================================================
				if ( concepts.size() == 1 && OpenCyc.getInstance().genlsSomethingExisting( concepts.get( 0 ) ) ) {
					assertion = "(#$isa #$" + objectInstance + " #$" + concepts.get( 0 ) + ")";
				}

				// ==================================================================
				// 2. Case: 2 Concepts, predicate and argument
				// ==================================================================
				else {
					String pred = null;

					// Search for a predicate
					for ( int j = 0; j < concepts.size(); j++ ) {
						if ( OpenCyc.getInstance().isaInUnivVocMt( concepts.get( j ), "Predicate" )
								&& OpenCyc.getInstance().getArity( concepts.get( j ) ) == 2 ) {
							pred = concepts.get( j );
						}
					}

					if ( pred != null ) {

						// Get type of argument 2
						String typeOfArg2 = OpenCyc.getInstance().getTypeOfArg( pred, 2 );
						if ( typeOfArg2 != null ) {

							// Remove all concepts that do not fit type of arg 2
							for ( int j = 0; j < concepts.size(); j++ ) {
								if ( ! OpenCyc.getInstance().isaInUnivVocMt( concepts.get( j ), typeOfArg2 )
										&& ! OpenCyc.getInstance().genlsInUniversalVocabularyMt( concepts.get( j ), typeOfArg2 ) ) {
									concepts.remove( j );
									j--;
									continue;
								}
							}

							assertion = "(#$" + pred + " #$" + objectInstance + " #$" + concepts.get( 0 ) + ")";
						}
					}

					else {
						// ==================================================================
						// 3. Case: Past Participle
						// ==================================================================

						// Search for an Event
						String event = null;
						for ( int j = 0; j < concepts.size(); j++ ) {
							if ( OpenCyc.getInstance().isaInUnivVocMt( concepts.get( j ), "TemporalObjectType" )
									|| OpenCyc.getInstance().isaInUnivVocMt( concepts.get( j ), "TemporalStuffType" )
									|| OpenCyc.getInstance().isaInUnivVocMt( concepts.get( j ), "ProcessType" )) {
								event = concepts.get( j );
							}
						}

						if ( event != null ) {

							String precondAction = makeInstanceOfAction( event, null );

							assertion = "(#$objectActedOn #$" + precondAction + " #$" + objectInstance + ")";
							OpenCyc.getInstance().makeAssertionInKitchenMt( assertion );
							
							assertion = "(#$preconditionFor-Events #$" + precondAction + " #$" + actionInstance + ")";
							OpenCyc.getInstance().makeAssertionInKitchenMt( assertion );
						}
					}
				}

				if ( assertion != null ) {
					OpenCyc.getInstance().makeAssertionInKitchenMt( assertion );
					assertions.add( assertion );
				}

			}
			catch ( Exception e ) {
				e.printStackTrace();
			}
		}
	}

	/**
	 * Converts a <code>Quantifier</code> of an Object into CycL and saves in the
	 * Cyc KB
	 * 
	 * @param q
	 * @param objectInstance
	 */
	public void convertQuantifier( Quantifier q, String objectInstance ) {

		// ==================================================================
		// Make out a maximum range of quantifying figures taking the first
		// and last one of the Quantifier, or an underspecified Quantity
		// ==================================================================
		List<String> range = new ArrayList<String>();

		for ( int i = 0; i < q.getAlternatives().size(); i++ ) {
			Word w = q.getAlternatives().get( i );

			// Check for a quotient
			if ( w.getLabel().split( "\\\\/" ).length > 1 ) {
				String[] tokens = w.getLabel().split( "\\\\/" );
				range.add( "(#$QuotientFn " + tokens[0] + " " + tokens[1] + ")" );
				continue;
			}

			// Check for underspecified amount
			if ( w.getCycConcepts().size() > 0 ) {
				range.add( "#$" + w.getCycConcepts().get( 0 ) );
				continue;
			}

			// Check for figures
			try {
				Double.valueOf( w.getLabel() );
				range.add( w.getLabel() );
			}
			catch ( Exception e ) {
				e.printStackTrace();
			}
		}

		if ( range.isEmpty() )
			return;

		String measure;

		if ( q.getMeasure().getCycConcepts().size() > 0 )
			measure = q.getMeasure().getCycConcepts().get( 0 );
		else
			measure = "Piece";

		String assertion = "(#$amountOfObject #$" + objectInstance + " (#$" + measure + " " + range.get( 0 )
				+ ( range.size() > 1 ? ( " " + range.get( range.size() - 1 ) ) : "" ) + "))";

		try {
			OpenCyc.getInstance().makeAssertionInKitchenMt( assertion );
			assertions.add( assertion );
		}
		catch ( Exception e ) {
			e.printStackTrace();
		}

	}

	/**
	 * Converts a <code>Preposition</code> that is nested, i.e. not directly hold
	 * by an <code>Instruction</code>, but by an <code>ObjectX</code>
	 * 
	 * @param pp
	 * @param parentObjects
	 * @throws CycApiException
	 * @throws UnknownHostException
	 * @throws IOException
	 */
	public void convertNestedPP( Preposition pp, List<String> parentObjects, String actionInstance ) throws CycApiException,
			UnknownHostException, IOException {

		List<String> objectInstances = null;

		// ==================================================================
		// Instantiate all objects of this Preposition
		// ==================================================================
		objectInstances = convertObjects( pp.getObjects(), actionInstance );

		// ==================================================================
		// Connect each parent Object with each child Object for each preposition
		// ==================================================================
		for ( int i = 0; i < objectInstances.size(); i++ ) {
			String child = objectInstances.get( i );

			for ( int k = 0; k < parentObjects.size(); k++ ) {
				String parent = parentObjects.get( k );

				for ( int j = 0; j < pp.getPrepositions().size(); j++ )
					connectObjectWithPreposition( child, parent, pp.getPrepositions().get( j ), null );

			}
		}
	}

	/**
	 * Resets the Converter instance
	 */
	public void resetConverter() {

		objectInstances.clear();
	}

	/**
	 * Returns all assertions that have been successfully made
	 * 
	 * @return
	 */
	public List<String> getAssertions() {

		return assertions;
	}

	public String getPlanName() {

		return planName;
	}
}
