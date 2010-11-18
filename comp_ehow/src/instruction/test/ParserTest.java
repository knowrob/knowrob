package instruction.test;

import instruction.disambiguator.Disambiguator;
import instruction.importer.PlanImporter;
import instruction.wrapper.LocalFileWrapper;
import java.io.File;

public class ParserTest {

	/**
	 * @param args
	 */
	public static void main( String[] args ) {
		
		PlanImporter importer = new PlanImporter();

		try {
			importer.initialize();
			System.out.println("Type the path to a HowTo:");
			while ( true ) {
				System.out.print(">");
				String s = "";
				int r;
				do {
					r = System.in.read();
					if (r != (char) '\n' && r != (char) '\r')
						s += (char) r;
				}
				while ( r != (char) '\n' );
				if (s.equals( "train" ))
					importer.getDisambiguator().setRunMode( Disambiguator.MODE_TRAIN );
				else if (s.equals( "apply" ))
					importer.getDisambiguator().setRunMode( Disambiguator.MODE_APPLY );
				else if (s.equals( "save" )) 
					importer.getDisambiguator().save( (new File(".")).getAbsolutePath() + File.separator + "etc/disambiguator.xml" );
				else if (s.equals( "load" ))
					importer.getDisambiguator().load( (new File(".")).getAbsolutePath() + File.separator + "etc/disambiguator.xml" );
				else {
					LocalFileWrapper wrapper = new LocalFileWrapper();
					wrapper.load( new File(".").getCanonicalPath() + File.separator + s );
					importer.importHowto( wrapper );
				}
			}
		}
		catch ( Exception e ) {
			e.printStackTrace();
		}
		

	}
}
