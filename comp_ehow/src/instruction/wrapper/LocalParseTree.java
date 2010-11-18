package instruction.wrapper;

import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

public class LocalParseTree implements IHowtoWebsiteWrapper {

	String title = "";
	List<String> instructions = new ArrayList<String>();
	String url = null;

	public String getHowtoTitle() {
		return title;
	}

	public List<String> getInstructions() {
		return instructions;
	}

	public void load( String url ) {
		File file = new File( url );
		
		this.url = url;

		try {
			// Read file to parse.
			String parseString = "";
			FileReader reader = new FileReader( file );
			int read = reader.read();

			while ( read >= 0 ) {
				parseString += (char) read;
				read = reader.read();
			}
			reader.close();

			// Parse every sentence since Standford Parser
			// can only cope with single sentences.
			parseString = parseString.replaceAll( "\r", "" );
			String[] sent = parseString.split( "\n\n" );
			instructions.clear();

			for ( int i = 0; i < sent.length; i++ ) {
				instructions.add( sent[i] );
			}
			
			// Get title of HowTo
			title = file.getName().replaceAll( "_", " " );
			
		}
		catch ( Exception e ) {
			e.printStackTrace();
		}
	}

	public String getUrl() {

		return url;
	}

}
