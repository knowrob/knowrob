package instruction.test;

import java.io.IOException;
import java.util.List;
import instruction.disambiguator.Disambiguator;
import instruction.factory.InstructionFactory;
import instruction.postprocessor.InstructionPostProcessor;
import instruction.semanticObjects.Instruction;
import instruction.syntaxparser.Parser;
import instruction.syntaxparser.SyntaxTree;

public class TestCommandLine {

	
	public static boolean ARG_PRINT_PARSER_TREE = true;
	
	/**
	 * @param args
	 */
	public static void main( String[] args ) {

		Parser parser = new Parser();
		InstructionFactory factory = new InstructionFactory( parser );
		Disambiguator disambiguator = new Disambiguator();
		InstructionPostProcessor postProc = new InstructionPostProcessor();

		System.out.println( "Initializing OpenCyc..." );
		System.out.println( "Ready for Services" );

		try {
			while ( true ) {
				System.out.print(">");
				String s = "";
				int r;
				do {
					r = System.in.read();
					s += (char) r;
				}
				while ( r != (char) '\n' );
				
				parser.usePreProcessor( Parser.PRE_PROCESSOR_QUOTATION_MARKS
						| Parser.PRE_PROCESSOR_USE_CIPHERS_ONLY );
				SyntaxTree t = parser.parse( s );
				
				if (ARG_PRINT_PARSER_TREE)
					t.printTree();
				
				try {
					
					System.out.print("Making Instructions...");
					List<Instruction> instructions = factory.makeInstructions( t );
					System.out.println("Done.");
					
					System.out.print("Retrieving Meanings of Words...");
		//			factory.findMeaningsForInstructions( instructions );
					System.out.println("Done.");
					
					System.out.print("Running Post-Processor...");
					postProc.run( instructions );
					System.out.println("Done.");
				
					System.out.print("Running Disambiguator...");
					disambiguator.disambiguateInstructions( instructions );
					System.out.println("Done.");
					
					// Print Instructions
					for (int i = 0; i < instructions.size(); i++) {
						System.out.println(instructions.get( i ));
					}
					
				}
				catch ( Exception e ) {
					System.out.println(e.getMessage());
					e.printStackTrace();
				}
				
				
			}
		}
		catch ( IOException e ) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
