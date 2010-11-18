package instruction.wordnet;

import instruction.exceptions.WordNetException;
import instruction.semanticObjects.Word;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.net.MalformedURLException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Map;
import java.util.Vector;
import jpl.Query;

/**
 * Provides access to the WordNet RDF database using the SWI Prolog Semantic Web
 * engine. In order to initialize the Prolog engine properly, the
 * <code>SWI_HOME_DIR</code> environment variable has to be set to the prolog
 * home directory.
 * 
 * @author Daniel Nyga
 * 
 */
public class WordNetRDF {

	/** rdf:type constant of a noun synset */
	public static final String RDF_TYPE_NOUN_SYNSET = "http://www.w3.org/2006/03/wn/wn20/schema/NounSynset";

	/** rdf:type constant of a verb synset */
	public static final String RDF_TYPE_VERB_SYNSET = "http://www.w3.org/2006/03/wn/wn20/schema/VerbSynset";

	/** rdf:type constant of an adjective synset */
	public static final String RDF_TYPE_ADJECTIVE_SYNSET = "http://www.w3.org/2006/03/wn/wn20/schema/AdjectiveSatelliteSynset";

	/** rdf:type constant of an adverb synset */
	public static final String RDF_TYPE_ADVERB_SYNSET = "http://www.w3.org/2006/03/wn/wn20/schema/AdverbSynset";

	public static final boolean CACHE_IRREGULAR_FORMS = true;

	protected static HashMap<String, String> irregularVerbs = null;

	protected static HashMap<String, String> irregularNouns = null;

	/**
	 * Prolog engine startup
	 */
//	static {
//		try {
//			Vector<String> args = new Vector<String>(Arrays.asList(Prolog
//					.get_default_init_args()));
//			args.add("-G128M");
//			// args.add("-q");
//			args.add("-nosignals");
//			Prolog.set_default_init_args(args.toArray(new String[0]));
//
//			// load the startup files and modules for WordNet
//			new Query("ensure_loaded('"
//					+ absoluteFilePath("etc/wordnet/wordnet.pl") + "')")
//					.oneSolution();
//			new Query("rdf_db:rdf_load('"
//					+ absoluteFilePath("etc/wordnet/wordnet-synset.rdf" + "')"))
//					.oneSolution();
//
//			cacheIrregularNouns();
//			cacheIrregularVerbs();
//
//		} catch (Exception e) {
//			e.printStackTrace();
//		}
//	}

	/**
	 * Returns the stem of the word <code>word</code> postulating the
	 * part-of-speech <code>pos</code>.
	 */
	public static String getWordStem(String word, String pos) {

		word = word.toLowerCase();

		String stem = null;

		boolean found = false;

		if (pos.equals(RDF_TYPE_VERB_SYNSET)) {
			stem = word;
			found = wordExactlyExistsAs(stem, pos);

			if (!found && word.endsWith("ed")) {
				stem = word.substring(0, word.length() - 1);
				found = wordExactlyExistsAs(stem, pos);
				if (!found) {
					stem = word.substring(0, word.length() - 2);
					found = wordExactlyExistsAs(stem, pos);
				}
			}

			if (!found) {
				if (irregularVerbs.get(word) != null) {
					stem = irregularVerbs.get(word);
					found = wordExactlyExistsAs(stem, pos);
				}
			}
		}

		else if (pos.equals(RDF_TYPE_ADVERB_SYNSET)) {
			stem = word;
			found = wordExactlyExistsAs(stem, pos);
		}

		else if (pos.equals(RDF_TYPE_ADJECTIVE_SYNSET)) {
			stem = word;
			found = wordExactlyExistsAs(stem, pos);
		}

		else if (pos.equals(RDF_TYPE_NOUN_SYNSET)) {
			stem = word;
			found = wordExactlyExistsAs(stem, pos);
			if (!found) {
				if (word.endsWith("s")) {
					stem = word.substring(0, word.length() - 1);
					found = wordExactlyExistsAs(stem, pos);
				}
				if (!found && word.endsWith("es")) {
					stem = word.substring(0, word.length() - 2);
					found = wordExactlyExistsAs(stem, pos);
				}
				if (!found) {
					if (irregularNouns.get(word) != null) {
						stem = irregularNouns.get(word);
						found = wordExactlyExistsAs(stem, pos);
					}
				}
			}
		}

		if (found)
			return stem;
		else
			return null;
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
	public static boolean wordExistsAs(String word, String pos) {

		boolean found = wordExactlyExistsAs(word, pos);

		if (!found) {

			String stem = getWordStem(word, pos);

			found = wordExactlyExistsAs(stem, pos);
		}

		return found;
	}

	public static String convertPOS(int type) {

		String pos = null;

		if (type == Word.TYPE_VERB_INFINITIVE
				|| type == Word.TYPE_PAST_PARTICIPLE
				|| type == Word.TYPE_GERUND)
			pos = RDF_TYPE_VERB_SYNSET;
		else if (type == Word.TYPE_NOUN)
			pos = RDF_TYPE_NOUN_SYNSET;
		else if (type == Word.TYPE_ADV)
			pos = RDF_TYPE_ADVERB_SYNSET;
		else if (type == Word.TYPE_ADJ)
			pos = RDF_TYPE_ADJECTIVE_SYNSET;

		return pos;
	}

	/**
	 * Checks whether the given word exists in WordNet data base, not
	 * considering different word forms.
	 * 
	 * @see WordNet.wordExistsAs(String, pos)
	 * 
	 * @param word
	 * @param pos
	 * @return
	 * @throws Exception
	 */
	public static boolean wordExactlyExistsAs(String word, String pos) {

		if (word == null)
			return false;

		return getSynsets(word, pos).size() > 0;
	}

	@SuppressWarnings("unused")
	private static void cacheIrregularNouns() throws IOException,
			WordNetException {
		File noun_exc = new File(absoluteFilePath("etc/wordnet/noun.exc"));
		if (!noun_exc.exists())
			throw new FileNotFoundException(
					"Mapping File for irregular Noun forms ("
							+ noun_exc.getAbsolutePath() + ") not found");
		else {
			FileReader reader = new FileReader(noun_exc);
			StringBuilder file = new StringBuilder();
			char[] buffer = new char[1024];
			int read = 0;
			while ((read = reader.read(buffer)) >= 0)
				file.append(buffer, 0, read);
			irregularNouns = new HashMap<String, String>();

			String[] mappings = file.toString().split(
					System.getProperty("line.separator"));
			for (int i = 0; i < mappings.length; i++) {
				String[] keyValueSet = mappings[i].split(" ");
				if (keyValueSet.length < 2)
					throw new WordNetException("Illegal Mapping entry in "
							+ noun_exc.getName());
				else {
					irregularNouns.put(keyValueSet[0], keyValueSet[1]);
				}
			}

		}
	}

	// public static void initializeMissingMappings() {
	//
	// missingNouns = new HashMap<String, String>();
	// missingNouns.put("stove burner", "04163507");
	// missingNouns.put("stove top", "TUM00001");
	// missingNouns.put("eating bowl", "02778691");
	// missingNouns.put("packet soup", "07115585");
	// missingNouns.put("contents", "07474342");
	// missingNouns.put("iced tea", "N07456631");
	//
	// missingVerbs = new HashMap<String, String>();
	// missingVerbs.put("look at", "TUM00002");
	// missingVerbs.put("bring out", "01145176");
	//
	// }

	public static String absoluteFilePath(String relativePath) {
		return new File(relativePath).getAbsolutePath().replaceAll("\\\\", "/");
	}

	/**
	 * Execute the given query. Assert the given premises and retract them after
	 * querying.
	 * 
	 * @param query
	 *            the query.
	 * @return the HashMap representing the bindings of the variables.
	 */
	private static Map<String, Vector<Object>> executeQuery(String query,
			String plFile) {

		// System.err.println("Executing query: " + query);

		HashMap<String, Vector<Object>> result = new HashMap<String, Vector<Object>>();
		Hashtable[] solutions;

		Query q = new Query("expand_goal((" + query + "),_9), call(_9)");

		// Due to bugs we have to check for one answer beforehand.
		if (!q.hasMoreSolutions())
			return new HashMap<String, Vector<Object>>();
		Hashtable oneSolution = q.nextSolution();
		if (oneSolution.isEmpty()) // Due to a bug consulting a file without
			// anything else results in shutdown
			return new HashMap<String, Vector<Object>>(); // I will try to
		// prevent it with
		// this construction

		// Restart the query and fetch everything.
		q.rewind();
		solutions = q.allSolutions();

		for (Object key : solutions[0].keySet()) {
			result.put(key.toString(), new Vector<Object>());
		}

		// Build the result
		for (int i = 0; i < solutions.length; i++) {
			Hashtable solution = solutions[i];
			for (Object key : solution.keySet()) {
				String keyStr = key.toString();

				if (!result.containsKey(keyStr)) {

					// previously unknown column, add result vector
					Vector<Object> resultVector = new Vector<Object>();
					resultVector.add(i, solution.get(key).toString());
					result.put(keyStr, resultVector);

				}
				// Put the solution into the correct vector
				Vector<Object> resultVector = result.get(keyStr);
				resultVector.add(i, solution.get(key).toString());
			}
		}
		// Generate the final QueryResult and return
		return result;
	}

	/**
	 * Query the WordNet database for synsets of the rdf:type <code>type</code>
	 * the word <code>word</code> belongs to. If <code>type</code> is null, all
	 * types of synsets are returned.
	 * 
	 * @see #RDF_TYPE_NOUN_SYNSET
	 * @see #RDF_TYPE_VERB_SYNSET
	 * @see #RDF_TYPE_ADJECTIVE_SYNSET
	 * @see #RDF_TYPE_ADVERB_SYNSET
	 */
	public static ArrayList<String> getSynsets(String word, String pos) {

		Map<String, Vector<Object>> results;

//		if (pos == null)
			results = executeQuery("comp_ehow:word_has_synset_uri('" + word.toLowerCase()
					+ "', Synsets)", null);
//		else
//			results = executeQuery("word_has_synset_of_type('"
//					+ word.toLowerCase() + "', Synsets, '" + pos + "')", null);

		Vector<Object> synsets = results.get("Synsets");

		if (synsets == null)
			return new ArrayList<String>();

		ArrayList<String> synsetArray = new ArrayList<String>();

		for (int i = 0; i < synsets.size(); i++)
			synsetArray.add(((String) synsets.get(i)).replaceAll("'", ""));

		return synsetArray;
	}

	public static String getSynsetFromID(String synsetID) {
		Map<String, Vector<Object>> results;

		results = executeQuery("comp_ehow:synset_from_id(Synset,'" + synsetID + "')",
				null);

		Vector<Object> synsets = results.get("Synset");

		if (synsets == null || synsets.size() == 0)
			return null;
		else
			return ((String)synsets.get(0)).replaceAll("'", "");
	}

	public static String readLine() {
		try {
			StringBuilder str = new StringBuilder();
			int read;
			read = System.in.read();

			while ((char) read != '\n') {
				if (((char) read) != '\r' && (char) read != '\n')
					str.append((char) read);

				read = System.in.read();
			}
			return str.toString();
		} catch (IOException e) {
			e.printStackTrace();
		}
		return null;
	}

	public static void main(String[] args) {
		//
		// WordNetRDF wn = new WordNetRDF();
		// Map<String, Vector<Object>> synsetIds = wn.executeQuery(
		// "word_has_synset_id('cup', SynsetId)", null);
		// for (String key : synsetIds.keySet()) {
		// System.out.println(key + " -> " + synsetIds.get(key).toString());
		// }
		//
		// String[] synsets = wn.getSynsets("cup", null);
		// for (String synset : synsets)
		// System.out.println(synset);
		//
		// Map<String, Vector<Object>> types = wn
		// .executeQuery(
		// "synset_type('http://www.w3.org/2006/03/wn/wn20/instances/synset-cup-noun-3', Type)",
		// null);
		// for (String key : types.keySet()) {
		// System.out.println(key + " -> " + types.get(key).toString());
		//
		// }
		
	//	System.out.println(getSynsetFromID("302351490"));

		do {
			String read = readLine();
			if (read.equals("exit"))
				break;
			if (wordExistsAs(read, RDF_TYPE_VERB_SYNSET)) {
				System.out.println(getSynsets(read, RDF_TYPE_VERB_SYNSET));
			}
		} while (true);

		// try {
		// if ( wordExistsAs( "chopped", Word.TYPE_PAST_PARTICIPLE ) )
		// System.out.println( "Word found" );
		// else
		// System.out.println( "Word not found" );
		// }
		// catch ( MalformedURLException e ) {
		// // TODO Auto-generated catch block
		// e.printStackTrace();
		// }
		// catch ( Exception e ) {
		// // TODO Auto-generated catch block
		// e.printStackTrace();
		// }
	}
}
