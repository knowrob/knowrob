package instruction.opencyc;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Iterator;
import org.opencyc.api.CycAccess;
import org.opencyc.api.CycApiException;
import org.opencyc.api.CycObjectFactory;
import org.opencyc.cycobject.CycConstant;
import org.opencyc.cycobject.CycFort;
import org.opencyc.cycobject.CycList;
import org.opencyc.cycobject.CycObject;
import org.opencyc.cycobject.CycVariable;

public class OpenCyc {

	public static final String MT_UNIVERSAL_VOCABULARY = "#$UniversalVocabularyMt";
	public static final String MT_TUM_KITCHEN_ENVIRONMENT = "#$TUMKitchenEnvironmentMt";
	public static final String MT_TUM_KITCHEN_ENVIRONMENT_PLANNING = "#$TUMKitchenEnvironmentPlanningMt";
	public static final String MT_TOPIC = "#$TopicMt";
	public static final String MT_WORDNET_MAPPING = "#$WordNetMappingMt";

	public static final String PRED_IS_EXECUTABLE_BY_ROBOT = "#$isExecutableByRobot";
	public static final String PRED_AMOUNT_OF_OBJECT = "#$amountOfObject";
	public static final String COLL_TYPICAL_OBJECT_IN_KITCHEN = "#$TypicalObjectInKitchen";

	public static final String MT_TUM_KITCHEN_ENVIRONMENT_COMMENT = "Microtheory containing specific Assertions for the TUM Kitchen Environment";
	public static final String MT_TUM_KITCHEN_ENVIRONMENT_PLANNING_COMMENT = "Microtheory containing Plans for the TUM Kitchen Environment";

	/** Interface to Cyc API */
	protected static CycAccess cyc = null;

	/** Singleton Instance */
	protected static OpenCyc me = null;

	/** Handle to <code>UniversalVocabularyMt</code> */
	protected CycObject universalVocMt = null;

	/** Handle to <code>TUMKitchenEnvironmentMt</code> */
	protected CycObject kitchenEnvMt = null;

	/** Handle to <code>TUMKitchenEnvirionmentPlanningMt</code> */
	protected CycObject kitchenPlanningMt = null;

	/** Handle to <code>TopicMt</code> */
	protected CycObject topicMt = null;

	/** Handle to <code>WordNetMappingMt</code> */
	protected CycObject wordNetMappingMt = null;

	/** Cyc Cache for better performance * */
	protected CycCache cache = null;

	protected OpenCyc () throws UnknownHostException, CycApiException, IOException {

		cyc = new CycAccess();
		makeCycAssertions();
	}

	/**
	 * Returns a Handle to <code>TUMKitchenEnvironmentMt</code>
	 * 
	 * @return
	 */
	public CycObject getKitchenEnvMt() {

		return kitchenEnvMt;
	}

	/**
	 * Returns a Handle to <code>TUMKitchenEnvironmentPlanningMt</code>
	 * 
	 * @return
	 */
	public CycObject getKitchenEnvPlanningMt() {

		return kitchenPlanningMt;
	}

	/**
	 * Returns a Handle to <code>UniversalVocabularyMt</code>
	 * 
	 * @return
	 */
	public CycObject getUniversalVocMt() {

		return universalVocMt;
	}

	
	/**
	 * Returns a Handle to <code>WordNetMappingMt</code>
	 * 
	 * @return
	 */
	public CycObject getWordNetMappingMt() {

		return wordNetMappingMt;
	}

	/**
	 * Initializes this <code>OpenCyc</code>-Class for usage
	 * 
	 * @throws UnknownHostException
	 * @throws CycApiException
	 * @throws IOException
	 */
	protected void initializeCyc() throws UnknownHostException, CycApiException, IOException, FileNotFoundException {

		universalVocMt = cyc.getConstantByName( MT_UNIVERSAL_VOCABULARY );
		kitchenEnvMt = cyc.getConstantByName( MT_TUM_KITCHEN_ENVIRONMENT );
		kitchenPlanningMt = cyc.getConstantByName( MT_TUM_KITCHEN_ENVIRONMENT_PLANNING );
		topicMt = cyc.getConstantByName( MT_TOPIC );
		wordNetMappingMt = cyc.getConstantByName( MT_WORDNET_MAPPING );
		
		// Initialize the Cyc concept cache
		cache = new CycCache();
		cache.load();
	}
	
	public void addMapping(String synset, String concept) throws UnknownHostException, CycApiException, IOException {
		CycList assertion = cyc.makeCycList( "(#$ist " + MT_WORDNET_MAPPING + " "
				+ "(#$synonymousExternalConcept #$" + concept + " #$WordNet-Version2_0 \"" + synset
				+ "\"))" );
		cyc.assertGaf( assertion, wordNetMappingMt );
	}

	/**
	 * Checks whether the given constant already exists in Cyc
	 * 
	 * @param constant
	 * @return
	 * @throws UnknownHostException
	 * @throws IOException
	 */
	public boolean constantExists( String constant ) throws UnknownHostException, IOException {

		try {
			if ( cyc.getConstantByName( constant ) != null ) {
				return true;
			}
			else {
				return false;
			}
		}
		catch ( CycApiException e ) {
			return false;
		}
	}

	/**
	 * Checks whether <code>upperConcept</code> is a generalisation (<code>genls</code>)
	 * of <code>subConcept</code> in <code>UniversalVocabularyMt</code>
	 * 
	 * @param subConcept
	 * @param upperConcept
	 * @return
	 * @throws UnknownHostException
	 * @throws CycApiException
	 * @throws IOException
	 */
	@SuppressWarnings("deprecation")
	public boolean genlsInUniversalVocabularyMt( String subConcept, String upperConcept )
			throws UnknownHostException, CycApiException, IOException {

		CycList query = cyc.makeCycList( "(#$ist #$UniversalVocabularyMt " + "(#$genls #$" + subConcept + " #$"
				+ upperConcept + "))" );

		return cyc.isQueryTrue( query, universalVocMt );
	}

	/**
	 * Checks whether <code>upperConcept</code> is a generalisation of the type (<code>typeGenls</code>)
	 * of <code>subConcept</code> in <code>UniversalVocabularyMt</code>
	 * 
	 * @param subConcept
	 * @param upperConcept
	 * @return
	 * @throws UnknownHostException
	 * @throws CycApiException
	 * @throws IOException
	 */
	@SuppressWarnings("deprecation")
	public boolean typeGenlsInUniversalVocabularyMt( String subConcept, String upperConcept )
			throws UnknownHostException, CycApiException, IOException {

		CycList query = cyc.makeCycList( "(#$ist #$UniversalVocabularyMt " + "(#$typeGenls #$" + subConcept
				+ " #$" + upperConcept + "))" );

		return cyc.isQueryTrue( query, universalVocMt );
	}

	/**
	 * Checks whether <code>upperConcept</code> is a generalisation (<code>genls</code>)
	 * of <code>subConcept</code> in <code>TopicMt</code>
	 * 
	 * @param subConcept
	 * @param upperConcept
	 * @return
	 * @throws UnknownHostException
	 * @throws CycApiException
	 * @throws IOException
	 */
	@SuppressWarnings("deprecation")
	public boolean genlsInTopicMt( String subConcept, String upperConcept ) throws UnknownHostException,
			CycApiException, IOException {

		CycList query = cyc.makeCycList( "(#$ist #$TopicMt (#$genls #$" + subConcept + " #$" + upperConcept
				+ "))" );

		return cyc.isQueryTrue( query, topicMt );
	}

	/**
	 * Makes the assertion <code>assertion</code> in the
	 * <code>TUMKitchenEnvirionmentMt</code>
	 * 
	 * @param assertion
	 * @throws CycApiException
	 * @throws UnknownHostException
	 * @throws IOException
	 */
	public void makeAssertionInKitchenMt( String assertion ) throws CycApiException, UnknownHostException,
			IOException {

		CycList list = OpenCyc.getInstance().getCycAcces().makeCycList(
				"(#$ist " + MT_TUM_KITCHEN_ENVIRONMENT + " " + assertion + ")" );
		System.out.println( assertion );
		OpenCyc.getInstance().getCycAcces().assertGaf( list, OpenCyc.getInstance().getKitchenEnvMt() );
	}

	/**
	 * Makes the assertion <code>assertion</code> in the
	 * <code>TUMKitchenEnvironmentPlanningMt</code>
	 * 
	 * @param assertion
	 * @throws CycApiException
	 * @throws UnknownHostException
	 * @throws IOException
	 */
	public void makeAssertionInKitchenPlanningMt( String assertion ) throws CycApiException,
			UnknownHostException, IOException {

		CycList list = OpenCyc.getInstance().getCycAcces().makeCycList(
				"(#$ist " + MT_TUM_KITCHEN_ENVIRONMENT_PLANNING + " " + assertion + ")" );
		System.out.println( assertion );
		OpenCyc.getInstance().getCycAcces().assertGaf( list, OpenCyc.getInstance().getKitchenEnvPlanningMt() );
	}

	/**
	 * Returns the type of the <code>arg</code>th argument of the predicate
	 * <code>pred</code>
	 * 
	 * @param pred
	 * @param arg
	 * @return
	 * @throws UnknownHostException
	 * @throws CycApiException
	 * @throws IOException
	 */
	@SuppressWarnings("deprecation")
	public String getTypeOfArg( String pred, int arg ) throws UnknownHostException, CycApiException,
			IOException {

		CycList query = cyc.makeCycList( "(#$ist #$UniversalVocabularyMt " + "(#$argIsa #$" + pred + " " + arg
				+ " ?TYPE))" );

		CycVariable var = CycObjectFactory.makeCycVariable( "?TYPE" );

		CycList ret = cyc.askWithVariable( query, var, universalVocMt );

		if ( ret.size() > 0 )
			return ret.get( 0 ).toString();

		return null;
	}

	/**
	 * Checks whether <code>subConcept</code> is an instance (<code>isa</code>)
	 * of <code>upperConcept</code> in <code>TopicMt</code>
	 * 
	 * @param subConcept
	 * @param upperConcept
	 * @return
	 * @throws UnknownHostException
	 * @throws CycApiException
	 * @throws IOException
	 */
	@SuppressWarnings("deprecation")
	public boolean isaInTopicMt( String subConcept, String upperConcept ) throws UnknownHostException,
			CycApiException, IOException {

		CycList query = cyc.makeCycList( "(#$ist #$TopicMt (#$isa #$" + subConcept + " #$" + upperConcept + "))" );

		return cyc.isQueryTrue( query, topicMt );
	}

	/**
	 * Retrieves a List of all Cyc Constants that are mapped via
	 * <code>synsetID</code> by the <code>synonymousExternalConcept</code>
	 * predicate in <code>WordNetMappingMt</code>
	 * 
	 * @param synsetID
	 * @return A List of Cyc Constant Strings
	 * @throws Exception 
	 */
	@SuppressWarnings({ "deprecation", "unchecked" })
	public ArrayList<String> getCycConceptFromWordNetID( String synsetID ) throws Exception {

		// Return the cached concepts, if existing
		ArrayList<String> concepts = (ArrayList) cache.ask( synsetID );
		
		if (concepts != null) {
			return concepts;
		}
		
		// Ask Cyc for the concepts
		CycList query = cyc.makeCycList( "(#$ist #$WordNetMappingMt "
				+ "(#$synonymousExternalConcept ?CONCEPT #$WordNet-Version2_0 \"" + synsetID + "\"))" );

		CycVariable var = CycObjectFactory.makeCycVariable( "?CONCEPT" );

		CycFort mt = cyc.getConstantByName( "#$WordNetMappingMt" );

		CycList ret = cyc.askWithVariable( query, var, mt );

		ArrayList<String> list = new ArrayList<String>();
		for ( Iterator i = ret.iterator(); i.hasNext(); ) {
			String str = i.next().toString();
			if ( str.indexOf( "(" ) == - 1 && str.indexOf( ")" ) == - 1 && str.indexOf( " " ) == - 1 )
				list.add( str );
		}
		
		// Cache the retrieved Cyc concepts
		cache.tell( synsetID, list );
		cache.save();
		
		return list;
	}

	public CycAccess getCycAcces() {

		return cyc;
	}

	/**
	 * Returns the singleton instance of the <code>ObjectCyc</code> Wrapper
	 * class
	 * 
	 * @return
	 * @throws UnknownHostException
	 * @throws CycApiException
	 * @throws IOException
	 */
	public static OpenCyc getInstance() throws UnknownHostException, CycApiException, IOException {

		if ( me == null ) {
			me = new OpenCyc20();
			me.initializeCyc();
		}
		return me;
	}

	/**
	 * Returns the arity of the predicate <code>predicate</code>
	 * 
	 * @param predicate
	 * @return
	 * @throws CycApiException
	 * @throws UnknownHostException
	 * @throws IOException
	 */
	public int getArity( String predicate ) throws CycApiException, UnknownHostException, IOException {

		int arity = cyc.getArity( cyc.getConstantByName( predicate ) );
		return arity;
	}

	/**
	 * Returns a list of all <code>reqiredActorSlots</code> of the concept
	 * <code>concept</code> in <code>UniversalVocabularyMt</code>
	 * 
	 * @param concept
	 * @return
	 * @throws UnknownHostException
	 * @throws CycApiException
	 * @throws IOException
	 */
	@SuppressWarnings("deprecation")
	public ArrayList<String> getRequiredActorSlots( String concept ) throws UnknownHostException,
			CycApiException, IOException {

		CycList query = cyc.makeCycList( "(#$ist #$UniversalVocabularyMt (#$requiredActorSlots #$" + concept
				+ " ?ACTORSLOT))" );
		CycVariable var = CycObjectFactory.makeCycVariable( "?ACTORSLOT" );
		CycFort mt = cyc.getConstantByName( "#$UniversalVocabularyMt" );

		CycList ret = cyc.askWithVariable( query, var, mt );

		// System.out.println("Size: " + ret.size());
		ArrayList<String> list = new ArrayList<String>();
		for ( Iterator i = ret.iterator(); i.hasNext(); ) {
			String str = i.next().toString();
			list.add( str );
		}

		return list;
	}

	/**
	 * Asks Cyc whether the Action action is executable, i.e. whether the
	 * isExecutableByRobot Predicate is set in TUMKitchenEnvironmentMt
	 * 
	 * @param action
	 *          Cyc concept of Action, without leading "#$"
	 * @return
	 * @throws IOException
	 * @throws CycApiException
	 * @throws UnknownHostException
	 */
	@SuppressWarnings("deprecation")
	public boolean isExecutableByRobot( String action ) throws UnknownHostException, CycApiException,
			IOException {

		CycList query = cyc.makeCycList( "(#$ist " + MT_TUM_KITCHEN_ENVIRONMENT + " ("
				+ PRED_IS_EXECUTABLE_BY_ROBOT + " #$" + action + "))" );

		return cyc.isQueryTrue( query, kitchenEnvMt );

	}

	/**
	 * Checks whether <code>concept</code> is an instance of
	 * <code>upperConcept</code> in <code>UniversalVocabularyMt</code>
	 * 
	 * @param concept
	 * @param genls
	 * @return
	 * @throws UnknownHostException
	 * @throws CycApiException
	 * @throws IOException
	 */
	@SuppressWarnings("deprecation")
	public boolean isaInUnivVocMt( String concept, String upperConcept ) throws UnknownHostException,
			CycApiException, IOException {

		CycList query = cyc.makeCycList( "(#$ist " + MT_UNIVERSAL_VOCABULARY + " (" + "#$isa" + " #$" + concept
				+ " " + "#$" + upperConcept + "))" );

		return cyc.isQueryTrue( query, universalVocMt );

	}

	/**
	 * Checks whether <code>concept</code> is a specialization of
	 * <code>TypicalObjectInKitchen</code> in
	 * <code>TUMKitchenEnvironmentMt</code>
	 * 
	 * @param concept
	 * @return
	 * @throws UnknownHostException
	 * @throws CycApiException
	 * @throws IOException
	 */
	@SuppressWarnings("deprecation")
	public boolean isTypicalObjectInKitchen( String concept ) throws UnknownHostException, CycApiException,
			IOException {

		CycList query = cyc.makeCycList( "(#$ist " + MT_TUM_KITCHEN_ENVIRONMENT + " (" + "#$isa" + " #$"
				+ concept + " " + COLL_TYPICAL_OBJECT_IN_KITCHEN + "))" );

		return cyc.isQueryTrue( query, universalVocMt );

	}

	/**
	 * Checks whether <code>concept</code> is a specialization of
	 * <code>SomethingExisting</code> in <code>UniversalVocabularyMt</code>
	 * 
	 * @param concept
	 * @return
	 * @throws UnknownHostException
	 * @throws CycApiException
	 * @throws IOException
	 */
	@SuppressWarnings("deprecation")
	public boolean genlsSomethingExisting( String concept ) throws UnknownHostException, CycApiException,
			IOException {

		CycList query = cyc.makeCycList( "(#$ist " + MT_UNIVERSAL_VOCABULARY + " (" + "#$genls" + " #$" + concept
				+ " #$SomethingExisting))" );

		return cyc.isQueryTrue( query, universalVocMt );

	}

	/**
	 * Adds the given Cyc Action Concept to the Actions, which are executable by
	 * the Robot, i.e. sets the isExecutableByRobot Predicate to true
	 * 
	 * @param action
	 *          Cyc concept of Action, without leading "#$"
	 * @return
	 * @throws IOException
	 * @throws CycApiException
	 * @throws UnknownHostException
	 */
	public void addExecutableByRobot( String action ) throws UnknownHostException, CycApiException, IOException {

		CycList assertion = cyc.makeCycList( "(#$ist " + MT_TUM_KITCHEN_ENVIRONMENT + "("
				+ PRED_IS_EXECUTABLE_BY_ROBOT + " #$" + action + "))" );
		cyc.assertGaf( assertion, kitchenEnvMt );
	}

	/**
	 * 
	 * @param object
	 * @throws UnknownHostException
	 * @throws CycApiException
	 * @throws IOException
	 */
	public void addTypicalObjectInKitchen( String object ) throws UnknownHostException, CycApiException,
			IOException {

		CycList assertion;

		assertion = cyc.makeCycList( "(#$ist " + MT_TUM_KITCHEN_ENVIRONMENT + "(#$isa #$" + object + " "
				+ COLL_TYPICAL_OBJECT_IN_KITCHEN + "))" );
		cyc.assertGaf( assertion, kitchenEnvMt );
	}

	/**
	 * Initializes Cyc with the new TUMKitchenEnvironmentMt and some assertions on
	 * predefined Actions (not for external usage, only for debugging purpose)
	 * 
	 * @deprecated
	 */
	private void makeCycAssertions() {

		/**
		 * Create TUMKitchenEnvironmentMt
		 */
		try {
			CycFort baseKB = cyc.getKnownConstantByName( "#$BaseKB" );

			System.out.println( "Creating " + MT_TUM_KITCHEN_ENVIRONMENT + " Microtheory..." );
			cyc.makeCycConstant( MT_TUM_KITCHEN_ENVIRONMENT );
			CycList assertion = cyc.makeCycList( "(#$ist #$BaseKB (#$isa " + MT_TUM_KITCHEN_ENVIRONMENT
					+ " #$Microtheory))" );
			cyc.assertGaf( assertion, baseKB );

			System.out.println( "Creating " + MT_TUM_KITCHEN_ENVIRONMENT_PLANNING + "..." );
			cyc.makeCycConstant( MT_TUM_KITCHEN_ENVIRONMENT_PLANNING );
			assertion = cyc.makeCycList( "(#$ist #$BaseKB (#$isa " + MT_TUM_KITCHEN_ENVIRONMENT_PLANNING
					+ " #$Microtheory))" );
			cyc.assertGaf( assertion, baseKB );

			kitchenEnvMt = cyc.getConstantByName( MT_TUM_KITCHEN_ENVIRONMENT );
			kitchenPlanningMt = cyc.getConstantByName( MT_TUM_KITCHEN_ENVIRONMENT_PLANNING );

			CycConstant univoc = cyc.getConstantByName( "#$UniversalVocabularyMt" );
			
			// ******************************************************************************
			// Create amountOfObject
			// ******************************************************************************
			cyc.makeCycConstant( PRED_AMOUNT_OF_OBJECT );
			assertion = cyc.makeCycList( "(#$ist #$UniversalVocabularyMt (#$isa " + PRED_AMOUNT_OF_OBJECT
					+ " #$UnderspecifiedPredicate))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc.makeCycList( "(#$ist #$UniversalVocabularyMt (#$arity " + PRED_AMOUNT_OF_OBJECT
					+ " 2))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc.makeCycList( "(#$ist #$UniversalVocabularyMt (#$argIsa " + PRED_AMOUNT_OF_OBJECT
					+ " 1 #$Thing))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc.makeCycList( "(#$ist #$UniversalVocabularyMt (#$argIsa " + PRED_AMOUNT_OF_OBJECT
					+ " 2 #$UnitOfMeasureConcept))" );
			cyc.assertGaf( assertion, univoc );

			// ******************************************************************************
			// Create Piece
			// ******************************************************************************
			cyc.makeCycConstant( "#$Piece" );
			assertion = cyc.makeCycList( "(#$ist #$UniversalVocabularyMt (#$isa #$Piece #$UnitOfMeasureConcept))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc.makeCycList( "(#$ist #$UniversalVocabularyMt (#$isa #$Piece #$UnitOfMeasure))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc
					.makeCycList( "(#$ist #$UniversalVocabularyMt (#$resultIsa #$Piece #$UnitOfMeasureConcept))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc.makeCycList( "(#$ist #$UniversalVocabularyMt (#$argsIsa #$Piece #$NumericInterval))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc.makeCycList( "(#$ist #$UniversalVocabularyMt (#$arityMax #$Piece 2))" );
			cyc.assertGaf( assertion, univoc );

			// ******************************************************************************
			// Create Drop-UnitOfVolume
			// ******************************************************************************
			cyc.makeCycConstant( "#$Drop-UnitOfVolume" );
			assertion = cyc
					.makeCycList( "(#$ist #$UniversalVocabularyMt (#$isa #$Drop-UnitOfVolume #$UnitOfMeasureConcept))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc
					.makeCycList( "(#$ist #$UniversalVocabularyMt (#$isa #$Drop-UnitOfVolume #$UnitOfVolume))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc
					.makeCycList( "(#$ist #$UniversalVocabularyMt (#$resultIsa #$Drop-UnitOfVolume #$UnitOfVolume))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc
					.makeCycList( "(#$ist #$UniversalVocabularyMt (#$argsIsa #$Drop-UnitOfVolume #$NumericInterval))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc.makeCycList( "(#$ist #$UniversalVocabularyMt (#$arityMax #$Drop-UnitOfVolume 2))" );
			cyc.assertGaf( assertion, univoc );
			
			// ******************************************************************************
			// Create Splash-UnitOfVolume
			// ******************************************************************************
			cyc.makeCycConstant( "#$Splash-UnitOfVolume" );
			assertion = cyc
					.makeCycList( "(#$ist #$UniversalVocabularyMt (#$isa #$Splash-UnitOfVolume #$UnitOfMeasureConcept))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc
					.makeCycList( "(#$ist #$UniversalVocabularyMt (#$isa #$Splash-UnitOfVolume #$UnitOfVolume))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc
					.makeCycList( "(#$ist #$UniversalVocabularyMt (#$resultIsa #$Splash-UnitOfVolume #$UnitOfVolume))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc
					.makeCycList( "(#$ist #$UniversalVocabularyMt (#$argsIsa #$Splash-UnitOfVolume #$NumericInterval))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc.makeCycList( "(#$ist #$UniversalVocabularyMt (#$arityMax #$Splash-UnitOfVolume 2))" );
			cyc.assertGaf( assertion, univoc );
			
			// ******************************************************************************
			// Create Pinch-UnitOfVolume
			// ******************************************************************************
			cyc.makeCycConstant( "#$Pinch-UnitOfVolume" );
			assertion = cyc
					.makeCycList( "(#$ist #$UniversalVocabularyMt (#$isa #$Pinch-UnitOfVolume #$UnitOfMeasureConcept))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc
					.makeCycList( "(#$ist #$UniversalVocabularyMt (#$isa #$Pinch-UnitOfVolume #$UnitOfVolume))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc
					.makeCycList( "(#$ist #$UniversalVocabularyMt (#$resultIsa #$Pinch-UnitOfVolume #$UnitOfVolume))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc
					.makeCycList( "(#$ist #$UniversalVocabularyMt (#$argsIsa #$Pinch-UnitOfVolume #$NumericInterval))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc.makeCycList( "(#$ist #$UniversalVocabularyMt (#$arityMax #$Pinch-UnitOfVolume 2))" );
			cyc.assertGaf( assertion, univoc );

			// ******************************************************************************
			// Create TeaBag
			// ******************************************************************************
			cyc.makeCycConstant( "#$TeaBag" );
			assertion = cyc.makeCycList( "(#$ist #$UniversalVocabularyMt (#$isa #$TeaBag #$ExistingObjectType))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc
					.makeCycList( "(#$ist #$UniversalVocabularyMt (#$isa #$TeaBag #$ArtifactTypeByGenericCategory))" );
			cyc.assertGaf( assertion, univoc );
			assertion = cyc.makeCycList( "(#$ist #$UniversalVocabularyMt (#$genls #$TeaBag #$ContainerArtifact))" );
			cyc.assertGaf( assertion, univoc );

			// ******************************************************************************
			// Create BoilingPoint
			// ******************************************************************************

			assertion = cyc
					.makeCycList( "(#$ist #$UniversalVocabularyMt (#$genls #$KeepingInACertainState #$Event))" );
			cyc.assertGaf( assertion, univoc );
			
			// ******************************************************************************
			// Create donot
			// ******************************************************************************
			cyc.makeCycConstant( "#$notDo" );
			assertion = cyc.makeCycList( "(#$isa #$notDo #$UnaryPredicate)" );
			cyc.assertGaf( assertion, kitchenPlanningMt );
			assertion = cyc.makeCycList( "(#$arg1Isa #$notDo #$SimpleActionPredicate)" );
			cyc.assertGaf( assertion, kitchenPlanningMt );
			
			cyc.makeCycConstant( "#$orDo" );
			assertion = cyc.makeCycList( "(#$isa #$orDo #$VariableArityPredicate)" );
			cyc.assertGaf( assertion, kitchenPlanningMt );
			assertion = cyc.makeCycList( "(#$argsIsa #$orDo #$SimpleActionPredicate)" );
			cyc.assertGaf( assertion, kitchenPlanningMt );
			
			universalVocMt = cyc.getConstantByName( MT_UNIVERSAL_VOCABULARY );

		}
		catch ( UnknownHostException e ) {
			e.printStackTrace();
		}
		catch ( CycApiException e ) {
			e.printStackTrace();
		}
		catch ( IOException e ) {
			e.printStackTrace();
		}

	}

}
