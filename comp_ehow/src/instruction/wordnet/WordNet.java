package instruction.wordnet;

import instruction.exceptions.WordNetException;
import instruction.semanticObjects.Word;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.net.MalformedURLException;
import java.net.URL;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import edu.mit.jwi.Dictionary;
import edu.mit.jwi.IDictionary;
import edu.mit.jwi.item.IIndexWord;
import edu.mit.jwi.item.IWordID;
import edu.mit.jwi.item.POS;

public class WordNet {

	public static final boolean CACHE_IRREGULAR_FORMS = true;

	private static IDictionary dict = null;

	private static HashMap<String, String> irregularVerbs = null;

	private static HashMap<String, String> irregularNouns = null;

	private static HashMap<String, String> missingNouns = null;

	private static HashMap<String, String> missingVerbs = null;

	/**
	 * Retrieves a list of Synset-IDs for the given word and type
	 * 
	 * @param word
	 * @param type
	 * @return
	 * @throws Exception
	 * @throws MalformedURLException
	 */
	public static ArrayList<String> getSynIDs( String word, int type ) throws Exception {

		if ( dict == null ) {
			getInstance();
		}

		try {
			ArrayList<String> synsetIDs = new ArrayList<String>();
			POS pos = convertPOS( type );

			if ( pos == null || word == null )
				return synsetIDs;

			IIndexWord idxWord = null;

			if ( type == Word.TYPE_PAST_PARTICIPLE ) {
				if ( word.endsWith( "ed" ) ) {
					idxWord = getInstance().getIndexWord( word.substring( 0, word.length() - 1 ), pos );
					if ( idxWord == null )
						idxWord = getInstance().getIndexWord( word.substring( 0, word.length() - 2 ), pos );
				}

				if ( idxWord == null )
					if ( irregularVerbs.get( word ) != null )
						idxWord = getInstance().getIndexWord( irregularVerbs.get( word ), pos );
			}

			if ( type == Word.TYPE_ADV ) {
				idxWord = getInstance().getIndexWord( word, pos );
			}

			if ( type == Word.TYPE_NOUN ) {
				String syn = missingNouns.get( word.toLowerCase() );
				if ( syn != null )
					synsetIDs.add( syn );
			}

			else if ( type == Word.TYPE_VERB_INFINITIVE ) {
				String syn = missingVerbs.get( word.toLowerCase() );
				if ( syn != null )
					synsetIDs.add( syn );
			}

			else if ( type == Word.TYPE_ADJ ) {
				idxWord = getInstance().getIndexWord( word, pos );
			}

			if ( idxWord == null ) {
				if ( type == Word.TYPE_NOUN ) {
					if ( word.endsWith( "s" ) ) {
						idxWord = getInstance().getIndexWord( word.substring( 0, word.length() - 1 ), pos );
					}
					if ( idxWord == null && word.endsWith( "es" ) ) {
						idxWord = getInstance().getIndexWord( word.substring( 0, word.length() - 2 ), pos );
					}
					if ( idxWord == null ) {
						idxWord = getInstance().getIndexWord( word, pos );
					}
					if ( idxWord == null ) {
						if ( irregularNouns.get( word ) != null )
							idxWord = getInstance().getIndexWord( irregularNouns.get( word ), pos );
					}
				}
				else if ( type == Word.TYPE_VERB_INFINITIVE ) {
					idxWord = getInstance().getIndexWord( word, pos );
					if ( idxWord == null ) {
						if ( irregularVerbs.get( word ) != null )
							idxWord = getInstance().getIndexWord( irregularVerbs.get( word ), pos );
					}
				}
			}

			if ( idxWord == null )
				return synsetIDs;

			else {
				List<IWordID> wordIDs = idxWord.getWordIDs();

				for ( Iterator<IWordID> i = wordIDs.iterator(); i.hasNext(); ) {
					IWordID wID = i.next();
					DecimalFormat df = new DecimalFormat( "00000000" );
					synsetIDs.add( df.format( wID.getSynsetID().getOffset() ) );
				}

				return synsetIDs;
			}
		}
		catch ( MalformedURLException e ) {
			throw new WordNetException( e.getMessage() );
		}
	}

	/**
	 * Checks whether the given word exists in WordNet data base, considering
	 * different word forms, e.g. plural forms for nouns or distinct verb forms.
	 * 
	 * @see WordNet.wordExactlyExistsAs(String, int)
	 * 
	 * @param word
	 * @param type
	 * @return
	 * @throws Exception
	 * @throws MalformedURLException
	 */
	public static boolean wordExistsAs( String word, int type ) throws Exception {

		boolean found = wordExactlyExistsAs( word, type );

		if ( ! found ) {
			switch ( type ) {
			case Word.TYPE_PAST_PARTICIPLE:

				if ( word.endsWith( "ed" ) ) {
					found = wordExactlyExistsAs( word.substring( 0, word.length() - 1 ), type );
					if ( ! found )
						found = wordExactlyExistsAs( word.substring( 0, word.length() - 2 ), type );
				}

				if ( ! found )
					found = wordExactlyExistsAs( irregularVerbs.get( word ), type );
				break;

			case Word.TYPE_GERUND:

				if ( word.endsWith( "ing" ) ) {
					found = wordExactlyExistsAs( word.substring( 0, word.length() - 3 ), type );
				}

				if ( ! found )
					found = wordExactlyExistsAs( irregularVerbs.get( word ), type );
				break;

			case Word.TYPE_NOUN:
				if ( word.endsWith( "s" ) ) {
					found = wordExactlyExistsAs( word.substring( 0, word.length() - 1 ), type );
				}
				if ( ! found && word.endsWith( "es" ) ) {
					found = wordExactlyExistsAs( word.substring( 0, word.length() - 2 ), type );
				}
				if ( ! found ) {
					found = wordExactlyExistsAs( irregularNouns.get( word ), type );
				}
				break;
			}
		}

		return found;
	}

	public static ArrayList<Integer> getPossiblePOS( String w ) throws Exception {

		ArrayList<Integer> pos = new ArrayList<Integer>();

		if ( wordExistsAs( w, Word.TYPE_NOUN ) )
			pos.add( Word.TYPE_NOUN );

		if ( wordExistsAs( w, Word.TYPE_VERB_INFINITIVE ) )
			pos.add( Word.TYPE_VERB_INFINITIVE );

		if ( wordExistsAs( w, Word.TYPE_ADJ ) )
			pos.add( Word.TYPE_ADJ );

		if ( wordExistsAs( w, Word.TYPE_ADV ) )
			pos.add( Word.TYPE_ADV );

		return pos;
	}

	private static POS convertPOS( int type ) {

		POS pos = null;

		if ( type == Word.TYPE_VERB_INFINITIVE || type == Word.TYPE_PAST_PARTICIPLE || type == Word.TYPE_GERUND )
			pos = POS.VERB;
		else if ( type == Word.TYPE_NOUN )
			pos = POS.NOUN;
		else if ( type == Word.TYPE_ADV )
			pos = POS.ADVERB;
		else if ( type == Word.TYPE_ADJ )
			pos = POS.ADJECTIVE;

		return pos;
	}

	/**
	 * Checks whether the given word exists in WordNet data base, not considering
	 * different word forms.
	 * 
	 * @see WordNet.wordExistsAs(String, int)
	 * 
	 * @param word
	 * @param type
	 * @return
	 * @throws Exception
	 */
	public static boolean wordExactlyExistsAs( String word, int type ) throws Exception {

		if ( word == null )
			return false;

		try {
			POS pos = convertPOS( type );
			if ( pos == null )
				return false;

			IIndexWord idxWord = getInstance().getIndexWord( word, pos );

			if ( idxWord == null ) {
				if ( type == Word.TYPE_NOUN ) {
					if ( missingNouns.get( word ) != null )
						return true;
				}
				else if ( type == Word.TYPE_VERB_INFINITIVE ) {
					if ( missingVerbs.get( word ) != null )
						return true;
				}
			}
			else
				return true;
			return false;
		}
		catch ( MalformedURLException e ) {
			throw new WordNetException( e.getMessage() );
		}
	}

	private static IDictionary getInstance() throws Exception {

		if ( dict == null ) {
			String wnhome = System.getenv( "WNHOME" );
			if ( wnhome == null )
				throw new Exception( "Environment variable \"WNHOME\" not set. WordNet could not be found." );
			String path = wnhome + File.separator + "dict";
			URL url = new URL( "file", null, path );
			dict = new Dictionary( url );
			if ( ! dict.isOpen() )
				dict.open();

			// Cache irregular verb and noun forms
			if ( CACHE_IRREGULAR_FORMS ) {
				cacheIrregularVerbs( path );
				cacheIrregularNouns( path );
			}
			initializeMissingMappings();
		}

		return dict;
	}

	private static void cacheIrregularVerbs( String wnPath ) throws IOException, WordNetException {

		File verb_exc = new File( wnPath + File.separator + "verb.exc" );
		if ( ! verb_exc.exists() )
			throw new FileNotFoundException( "Mapping File for irregular Verb forms (" + verb_exc.getAbsolutePath()
					+ ") not found" );
		else {
			FileReader reader = new FileReader( verb_exc );
			StringBuilder file = new StringBuilder();
			char[] buffer = new char[1024];
			int read = 0;
			while ( ( read = reader.read( buffer ) ) >= 0 )
				file.append( buffer, 0, read );
			irregularVerbs = new HashMap<String, String>();

			String[] mappings = file.toString().split( System.getProperty( "line.separator" ) );
			for ( int i = 0; i < mappings.length; i++ ) {
				String[] keyValueSet = mappings[i].split( " " );
				if ( keyValueSet.length < 2 )
					throw new WordNetException( "Illegal Mapping entry in " + verb_exc.getName() + " -> "
							+ keyValueSet.length + " Tokens " + i );
				else {
					irregularVerbs.put( keyValueSet[0], keyValueSet[1] );
				}
			}

		}
	}

	private static void cacheIrregularNouns( String wnPath ) throws IOException, WordNetException {

		File noun_exc = new File( wnPath + File.separator + "noun.exc" );
		if ( ! noun_exc.exists() )
			throw new FileNotFoundException( "Mapping File for irregular Noun forms (" + noun_exc.getAbsolutePath()
					+ ") not found" );
		else {
			FileReader reader = new FileReader( noun_exc );
			StringBuilder file = new StringBuilder();
			char[] buffer = new char[1024];
			int read = 0;
			while ( ( read = reader.read( buffer ) ) >= 0 )
				file.append( buffer, 0, read );
			irregularNouns = new HashMap<String, String>();

			String[] mappings = file.toString().split( System.getProperty( "line.separator" ) );
			for ( int i = 0; i < mappings.length; i++ ) {
				String[] keyValueSet = mappings[i].split( " " );
				if ( keyValueSet.length < 2 )
					throw new WordNetException( "Illegal Mapping entry in " + noun_exc.getName() );
				else {
					irregularNouns.put( keyValueSet[0], keyValueSet[1] );
				}
			}

		}
	}

	public static void initializeMissingMappings() {

		missingNouns = new HashMap<String, String>();
		missingNouns.put( "stove burner", "04163507" );
		missingNouns.put( "stove top", "TUM00001" );
		missingNouns.put( "eating bowl", "02778691" );
		missingNouns.put( "packet soup", "07115585" );
		missingNouns.put( "contents", "07474342" );
		missingNouns.put( "iced tea", "N07456631" );

		missingVerbs = new HashMap<String, String>();
		missingVerbs.put( "look at", "TUM00002" );
		missingVerbs.put( "bring out", "01145176" );

	}

	public static void main( String[] args ) {

		try {
			if ( wordExistsAs( "chopped", Word.TYPE_PAST_PARTICIPLE ) )
				System.out.println( "Word found" );
			else
				System.out.println( "Word not found" );
		}
		catch ( MalformedURLException e ) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		catch ( Exception e ) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
