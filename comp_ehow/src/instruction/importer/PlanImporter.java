package instruction.importer;

import instruction.converter.Instruction2CycLConverter;
import instruction.disambiguator.Disambiguator;
import instruction.exceptions.InstructionException;
import instruction.exporter.PlanExporter;
import instruction.factory.InstructionFactory;
import instruction.opencyc.OpenCyc20;
import instruction.postprocessor.InstructionPostProcessor;
import instruction.semanticObjects.Instruction;
import instruction.syntaxparser.Parser;
import instruction.syntaxparser.SyntaxTree;
import instruction.wordnet.WordNetRDF2;
import instruction.wrapper.IHowtoWebsiteWrapper;

import java.io.IOException;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import org.opencyc.api.CycApiException;

public class PlanImporter {

	// System components
	Parser parser = null;
	Disambiguator disambiguator = null;
	InstructionFactory factory = null;
	InstructionPostProcessor postProc = null;
	PlanExporter exporter = null;
	Instruction2CycLConverter converter = null;

	// Internal store for the generated data structures
	private List<SyntaxTree> trees = null;
	private List<Instruction> instructionList = null;
	private Instruction instTitle = null;
	private SyntaxTree parsedTitle = null;
	private IHowtoWebsiteWrapper wrapper = null;
	private String rplPlan = null;
	private String planName = null;
	private List<String> nlInst = null;

	// Progress Listener registry
	private List<InstructionProgressListener> progressListeners = new ArrayList<InstructionProgressListener>();

	/**
	 * Initiailizes the plan importer
	 * 
	 * @throws UnknownHostException
	 * @throws CycApiException
	 * @throws IOException
	 */
	public void initialize() throws Exception {

		notifyProgressListeners( 0, "Initializing Parser..." );
		if ( parser == null )
			parser = new Parser();
		notifyProgressListeners( 20, "Done.\n" );

		notifyProgressListeners( 20, "Initializing Disambiguator..." );
		if ( disambiguator == null )
			disambiguator = new Disambiguator();
		notifyProgressListeners( 40, "Done.\n" );

		notifyProgressListeners( 40, "Initializing InstructionFactory..." );
		if ( factory == null )
			factory = new InstructionFactory( parser );
		notifyProgressListeners( 60, "Done.\n" );

		notifyProgressListeners( 60, "Initializing Post-Processor..." );
		if ( postProc == null )
			postProc = new InstructionPostProcessor();
		notifyProgressListeners( 80, "Done.\n" );

		notifyProgressListeners( -1, "Initializing Cyc Ontology..." );
		OpenCyc20.getInstance();
		notifyProgressListeners( 85, "Done.\n" );
		
		notifyProgressListeners( -1, "Initializing WordNet 2.0..." );
		new WordNetRDF2();
		notifyProgressListeners( 85, "Done.\n" );

		notifyProgressListeners( -1, "Initializing Instruction2CycL-Converter..." );
		if ( converter == null )
			converter = new Instruction2CycLConverter();
		notifyProgressListeners( 90, "Done.\n" );

		notifyProgressListeners( -1, "Initializing RPL Plan Exporter..." );
		if ( exporter == null )
			exporter = PlanExporter.getInstance();
		notifyProgressListeners( 100, "Done.\n" );
	}

	public void setWrapper( IHowtoWebsiteWrapper wrp ) {

		wrapper = wrp;
	}

	/**
	 * Adds an implementation of the <code>InstructionProgressListener</code>
	 * interface to the set of listeners that will be notified about the
	 * transformation progress
	 * 
	 * @param listener
	 */
	public void addProgressListener( InstructionProgressListener listener ) {

		progressListeners.add( listener );
	}
	

	/**
	 * Removes the listener <code>listener</code> from the set of progress
	 * listeners
	 * 
	 * @param listener
	 */
	public void removeProgressListener( InstructionProgressListener listener ) {

		progressListeners.remove( listener );
	}

	/**
	 * Sends a notification event to all registered progress listeners
	 * 
	 * @param percent
	 * @param msg
	 */
	private void notifyProgressListeners( int percent, String msg ) {

		for ( Iterator<InstructionProgressListener> i = progressListeners.iterator(); i.hasNext(); ) {
			i.next().progressNotification( percent, msg );
		}
	}

	/**
	 * Returns the disambiguator component
	 * 
	 * @return
	 */
	public Disambiguator getDisambiguator() {

		return disambiguator;
	}

	/**
	 * Imports the Howto given by the <code>IHowtoWebsiteWrapper</code> wrapper
	 * 
	 * @param wrapper
	 */
	public void importHowto( IHowtoWebsiteWrapper wrapper ) {

		String howtoTitle = wrapper.getHowtoTitle();
		List<String> nlInst = wrapper.getInstructions();

		ArrayList<Instruction> howto = new ArrayList<Instruction>();

		for ( Iterator<String> iter = nlInst.iterator(); iter.hasNext(); ) {
			String sentence = iter.next();

			for ( int k = 0; k <= Parser.PRE_PROCESSOR_LOWER_CASE; k += Parser.PRE_PROCESSOR_LOWER_CASE ) {
				parser.usePreProcessor( Parser.PRE_PROCESSOR_QUOTATION_MARKS | Parser.PRE_PROCESSOR_USE_CIPHERS_ONLY
						| k );

				SyntaxTree tree = parser.parse( sentence );
				trees.add( tree );

				try {
					List<Instruction> instructions = factory.makeInstructions( tree );
					
					howto.addAll( instructions );
					
					for(Instruction instr : instructions) {
						instr.setNLSentence(sentence);
					}

					if ( ! instructions.isEmpty() )
						break;
				}
				catch ( Exception e ) {
					e.printStackTrace();
					continue;
				}
			}
		}
		notifyProgressListeners( 30, "Parsed." );
		notifyProgressListeners( 50, "Instructions recognized." );

		for ( int i = 0; i < howto.size(); i++ )
			System.out.println( howto.get( i ) );

		try {
			postProc.run( howto );
		}
		catch ( Exception e2 ) {
			e2.printStackTrace();
		}

		for ( int i = 0; i < howto.size(); i++ )
			System.out.println( howto.get( i ) );

		try {
			disambiguator.disambiguateInstructions( howto );
			instructionList = howto;
		}
		catch ( Exception e1 ) {
			e1.printStackTrace();
		}
		notifyProgressListeners( 70, "Instructions disambiguated." );

		for ( int i = 0; i < howto.size(); i++ )
			System.out.println( howto.get( i ) );

		// Get title of HowTo
		String title = "Then " + howtoTitle;
		SyntaxTree t = parser.parse( title );
		t.printTree();
		try {
			List<Instruction> titles = factory.makeInstructions( t );

			postProc.run( titles );
			disambiguator.disambiguateInstructions( titles );

			if ( titles.size() == 0 || titles.size() > 2 )
				throw new InstructionException( "Title of HowTo could not be resolved." );
			System.out.println( titles.get( 0 ) );
			converter.convertHowTo( howto, titles.get( 0 ) );
			notifyProgressListeners( 90, "Title resolved." );
		}
		catch ( Exception e ) {
			System.out.println( e.getMessage() );
			e.printStackTrace();
		}

	}

	public void parseInstructions() throws InstructionException {

		if ( wrapper == null )
			throw new InstructionException( "Howto undefined." );

		String howtoTitle = wrapper.getHowtoTitle();
		nlInst = wrapper.getInstructions();

		trees = new ArrayList<SyntaxTree>();

		parser.usePreProcessor( Parser.PRE_PROCESSOR_QUOTATION_MARKS | Parser.PRE_PROCESSOR_USE_CIPHERS_ONLY );

		notifyProgressListeners( 0, "Parsing Instructions...\n" );

		// Parse instructions
		int counter = 1;
		for ( Iterator<String> iter = nlInst.iterator(); iter.hasNext(); ) {
			notifyProgressListeners( (int) Math.round(100 * (double) counter / (double) nlInst.size() ), String.format(
					"%d of %d...", counter, nlInst.size() ) );
			String sentence = iter.next();
			SyntaxTree tree = parser.parse( sentence );
			trees.add( tree );
			notifyProgressListeners( -1, "Done.\n" );
			counter++;
		}

		notifyProgressListeners( -1, "Parsing title of Howto..." );
		parsedTitle = parser.parse( "Then " + howtoTitle );
		notifyProgressListeners( 100, "Done.\n" );
	}

	public void recognizeAndDisambiguateInstructions() throws Exception {

		if ( disambiguator == null || postProc == null )
			throw new InstructionException( "PlanImporter is not initialized." );

		if ( trees == null || trees.isEmpty()/* || parsedTitle == null*/ )
			throw new InstructionException( "There are no parsed instructions" );

		instructionList = new ArrayList<Instruction>();

		notifyProgressListeners( 0, "Recognizing Instructions..." );
		int sentNum = 0;
		for ( Iterator<SyntaxTree> i = trees.iterator(); i.hasNext(); ) {
			
			List<Instruction> instructions = factory.makeInstructions( i.next() );
			
			
			instructionList.addAll( instructions );
			for (Instruction in: instructions) {
				System.out.println(in);
				in.setNLSentence(nlInst.get(sentNum));
			}
			sentNum++;
		}
		notifyProgressListeners( 40, "Done.\n" );

		try {
			notifyProgressListeners( 50, "Running Post-Processor..." );
			postProc.run( instructionList );

			notifyProgressListeners( 70, "Disambiguating Instructions..." );
			disambiguator.disambiguateInstructions( instructionList );
			notifyProgressListeners( 100, "Done.\n" );
			for (Iterator i = instructionList.iterator(); i.hasNext(); )
				System.out.println(i.next());
		}
		
		catch ( Exception e2 ) {
			e2.printStackTrace();
		}
	}

	public void convert2CycAssertions() throws CycApiException, UnknownHostException, IOException,
			InstructionException {

		if ( converter == null )
			throw new InstructionException( "PlanImporter not initialized." );

		if ( instructionList == null/* || instTitle == null*/ )
			throw new InstructionException( "No instructions found to convert." );

		notifyProgressListeners( 50, "Importing into Cyc Knowledge Base..." );
		converter.convertHowTo( instructionList, instTitle );
		notifyProgressListeners( 100, "Done.\n" );
		
		planName = converter.getPlanName();
	}

	public void generateRPLPlan() throws InstructionException {

		if ( exporter == null )
			throw new InstructionException( "PlanImporter not initialized." );

		notifyProgressListeners( 50, "Generating RPL Plan..." );
		rplPlan = exporter.exportPlanToRPL( planName );
		notifyProgressListeners( 100, "Done.\n" );
	}

	public List<SyntaxTree> getSyntaxTrees() {

		return trees;
	}

	public List<Instruction> getInstructions() {

		return instructionList;
	}

	public SyntaxTree getParsedTitle() {

		return parsedTitle;
	}

	public Instruction getTitle() {

		return instTitle;
	}

	public List<String> getAssertions() {

		if (converter == null)
			return new ArrayList<String>();
		
		return converter.getAssertions();
	}

	public String getRPLPlan() {

		return rplPlan;
	}

	public IHowtoWebsiteWrapper getWrapper() {

		return wrapper;
	}
	
	public void setAddMappingListener(AddCycMappingListener listener) {
		postProc.setAddMappingListener(listener);
	}
}
