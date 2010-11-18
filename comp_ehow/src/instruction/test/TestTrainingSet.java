package instruction.test;

import java.io.File;
import java.util.ArrayList;
import java.util.Iterator;
import instruction.semanticObjects.Instruction;
import instruction.syntaxparser.Parser;

public class TestTrainingSet {

	/**
	 * @param args
	 */
	public static void main( String[] args ) {

		Parser parser = new Parser();
	//	InstructionFactory factory = new InstructionFactory( parser );
	//	Disambiguator disambiguator = new Disambiguator();
	//	InstructionPostProcessor postProc = new InstructionPostProcessor();

		File howtos = new File( "howtos" );

		if ( howtos.isDirectory() ) {
			String[] files = howtos.list();
			for ( int i = 0; i < files.length; i++ ) {
				File tmp = new File( files[i] );
				if ( tmp.isDirectory() )
					continue;

				parser.setFileToParse( howtos.getName() + File.separator + files[i] );

				ArrayList<Instruction> howto = new ArrayList<Instruction>();

				for ( Iterator<String> iter = parser.iterator(); iter.hasNext(); ) {
					String sentence = iter.next();
			//		System.out.println( sentence );

					for ( int k = 0; k <= Parser.PRE_PROCESSOR_LOWER_CASE; k += Parser.PRE_PROCESSOR_LOWER_CASE ) {
				//		parser.usePreProcessor( Parser.PRE_PROCESSOR_QUOTATION_MARKS
				//				| Parser.PRE_PROCESSOR_USE_CIPHERS_ONLY | k );

						//SyntaxTree tree = 
							parser.parse( sentence );
						// tree.printTree();
						try {
				//			List<Instruction> instructions = factory.makeInstructions( tree );
				//			factory.findMeaningsForInstructions( instructions );
				//			postProc.run( instructions );
							// disambiguator.disambiguateInstructions( instructions );
				//			howto.addAll( instructions );
				//			if ( ! instructions.isEmpty() )
				//				break;
						}
						catch ( Exception e ) {
							System.out.println( e.getMessage() );
							e.printStackTrace();
							continue;
						}
					}

				}
				System.out.println( "***************************\nHOW TO "
						+ files[i].replaceAll( "_", " " ).toUpperCase() );
				for ( int j = 0; j < howto.size(); j++ )
					System.out.println( howto.get( j ) );

			}
		}

	}

}
