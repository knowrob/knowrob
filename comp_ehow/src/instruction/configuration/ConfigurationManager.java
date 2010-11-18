package instruction.configuration;

import instruction.disambiguator.Disambiguator;
import instruction.wordnet.WordNetRDF;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.jdom.Document;
import org.jdom.Element;
import org.jdom.input.SAXBuilder;
import org.jdom.output.XMLOutputter;

public class ConfigurationManager {

	private static String pathDisambiguator = null;

	private static String pathCycCache = null;

	private static String pathHowtos = null;
	
	private static String pathKnowRob = null;
	
	private static int disambMode = Disambiguator.MODE_APPLY;

	private static String TAG_SETTINGS = "settings";

	private static String TAG_SETTING = "setting";

	private static String SETTING_DISAMBIGUATOR_PATH = "disambiguator";

	private static String SETTING_CYCCACHE_PATH = "cyccache";

	private static String SETTING_HOWTOS_PATH = "howtos";
	
	private static String SETTING_KNOWROB_PATH = "knowrob";

	private static String ATTR_SETTING = "name";

	private static String ATTR_VALUE = "value";

	private static String TAG_MAPPING = "mapping";

	private static Map<String, List<String>> mappings = new HashMap<String, List<String>>();

	/**
	 * Loads the settings from the local file <code>etc/settings.xml</code>
	 */
	@SuppressWarnings("unchecked")
	public static void loadSettings() {

		File file = new File( "/work/ros/tumros-internal/stacks/knowrob/comp_ehow/etc/settings.xml" );
		if ( ! file.exists() )
			return;

		try {
			FileReader reader = new FileReader( file );

			SAXBuilder sax = new SAXBuilder();
			Document doc;

			doc = sax.build( reader );

			Element cache = doc.getRootElement();

			List<Element> synsetids = cache.getChildren();

			// ==================================================================
			// Iterate over all setting entries
			// ==================================================================
			for ( Iterator<Element> i = synsetids.iterator(); i.hasNext(); ) {

				Element tag = i.next();

				if ( tag.getName().equalsIgnoreCase( TAG_SETTING ) ) {
					String setting = tag.getAttributeValue( ATTR_SETTING );
					String value = tag.getAttributeValue( ATTR_VALUE );

					if ( setting == null || value == null )
						continue;

					if ( setting.equalsIgnoreCase( SETTING_DISAMBIGUATOR_PATH ) ) {
						pathDisambiguator = value;
					}
					else if ( setting.equalsIgnoreCase( SETTING_CYCCACHE_PATH ) ) {
						pathCycCache = value;
					}
					else if ( setting.equalsIgnoreCase( SETTING_HOWTOS_PATH ) ) {
						pathHowtos = value;
					}
					else if ( setting.equalsIgnoreCase( SETTING_KNOWROB_PATH ) ) {
						pathKnowRob = value;
					}
				}
				else if ( tag.getName().equalsIgnoreCase( TAG_MAPPING ) ) {
					String synset = tag.getAttributeValue( ATTR_SETTING );
					String concept = tag.getAttributeValue( ATTR_VALUE );
					if (synset == null || concept == null)
						continue;
					
					List<String> concepts = mappings.get( synset );
					if (concepts == null) {
						concepts = new ArrayList<String>();
						mappings.put(synset, concepts);
					}
					concepts.add( concept );
				}
			}
		}

		catch ( Exception e ) {
			e.printStackTrace();
		}
	}

	/**
	 * Saves the settings to the local file <code>etc/settings.xml</code>
	 */
	public static void saveSettings() {

		try {
			File file = new File( "/work/ros/tumros-internal/stacks/knowrob/comp_ehow/etc/settings.xml" );
			if ( ! file.exists() )
				file.createNewFile();

			Element settings = new Element( TAG_SETTINGS );

			if ( pathDisambiguator != null ) {
				Element settingDisamb = new Element( TAG_SETTING );
				settingDisamb.setAttribute( ATTR_VALUE, pathDisambiguator );
				settingDisamb.setAttribute( ATTR_SETTING, SETTING_DISAMBIGUATOR_PATH );
				settings.addContent( settingDisamb );
			}
			if ( pathCycCache != null ) {
				Element settingCache = new Element( TAG_SETTING );
				settingCache.setAttribute( ATTR_VALUE, pathCycCache );
				settingCache.setAttribute( ATTR_SETTING, SETTING_CYCCACHE_PATH );
				settings.addContent( settingCache );
			}
			if ( pathKnowRob != null ) {
				Element settingCache = new Element( TAG_SETTING );
				settingCache.setAttribute( ATTR_VALUE, pathKnowRob );
				settingCache.setAttribute( ATTR_SETTING, SETTING_CYCCACHE_PATH );
				settings.addContent( settingCache );
			}
			if ( pathHowtos != null ) {
				Element settingHowtos = new Element( TAG_SETTING );
				settingHowtos.setAttribute( ATTR_VALUE, pathHowtos );
				settingHowtos.setAttribute( ATTR_SETTING, SETTING_HOWTOS_PATH );
				settings.addContent( settingHowtos );
			}
			
			Set<String> synsets = mappings.keySet();
			for (Iterator<String> i = synsets.iterator(); i.hasNext(); ) {
				String synset = i.next();
				List<String> concepts = mappings.get( synset );
				for (Iterator<String> j = concepts.iterator(); j.hasNext();) {
					Element mapping = new Element(TAG_MAPPING);
					String concept = j.next();
					mapping.setAttribute( ATTR_SETTING, synset );
					mapping.setAttribute( ATTR_VALUE, concept );
					settings.addContent( mapping );
				}
			}

			// ==================================================================
			// Create XML document and write it to file
			// ==================================================================
			Document doc = new Document( settings );

			XMLOutputter outPutter = new XMLOutputter( " ", true );
			FileWriter outStream;
			outStream = new FileWriter( file );
			outPutter.output( doc, outStream );
		}
		catch ( IOException e ) {
			e.printStackTrace();
		}

	}
	
	public static Map<String, List<String>> getMappings() {

		return mappings;
	}
	
	public static void setMappings( Map<String, List<String>> mappings ) {

		ConfigurationManager.mappings = mappings;
	}

	public static String getPathCycCache() {

		return pathCycCache;
	}

	public static String getPathDisambiguator() {

		return pathDisambiguator;
	}

	public static void setPathCycCache( String pathCycCache ) {

		ConfigurationManager.pathCycCache = pathCycCache;
	}

	public static void setPathDisambiguator( String pathDisambiguator ) {

		ConfigurationManager.pathDisambiguator = pathDisambiguator;
	}
	
	public static String getPathHowtos() {

		return pathHowtos;
	}
	
	public static void setPathHowtos( String pathHowtos ) {

		ConfigurationManager.pathHowtos = pathHowtos;
	}
	
	public static int getDisambiguatorMode() {
		return disambMode;
	}
	
	public static String getPathKnowRob() {
		return pathKnowRob;
	}
	
	public static void setPathKnowRob(String pathKnowRob) {
		ConfigurationManager.pathKnowRob = pathKnowRob;
	}
	
	public static void setDisambiguatorMode(int mode) {
		if (mode != Disambiguator.MODE_APPLY && mode != Disambiguator.MODE_TRAIN)
			return;
		disambMode = mode;
	}
	
	@SuppressWarnings("unchecked")
	public static void convertSynsets() {

		File file = new File( "etc/settings.xml" );
		if ( ! file.exists() )
			return;

		try {
			FileReader reader = new FileReader( file );

			SAXBuilder sax = new SAXBuilder();
			Document doc;

			doc = sax.build( reader );

			Element cache = doc.getRootElement();

			List<Element> synsetids = cache.getChildren();

			// ==================================================================
			// Iterate over all setting entries
			// ==================================================================
			for ( Iterator<Element> i = synsetids.iterator(); i.hasNext(); ) {

				Element tag = i.next();

				if ( tag.getName().equalsIgnoreCase( TAG_MAPPING ) ) {
					String synset = tag.getAttributeValue( ATTR_SETTING );
					String concept = tag.getAttributeValue( ATTR_VALUE );
					if (synset == null || concept == null)
						continue;
					
					System.out.print("processing " + synset);
					String synsetURI = null;
					if (synset.charAt(0) == 'N') {
						String newSynsetID = synset.replace('N', '1');
						System.out.print("->" + newSynsetID);
						System.out.println("->" + (synsetURI = WordNetRDF.getSynsetFromID(newSynsetID)));
					}
					else if (synset.charAt(0) == 'V') {
						String newSynsetID = synset.replace('V', '2');
						System.out.print("->" + newSynsetID);
						System.out.println("->" + (synsetURI = WordNetRDF.getSynsetFromID(newSynsetID)));
					}
					else if (synset.charAt(0) == 'A') {
						String newSynsetID = synset.replace('A', '3');
						System.out.print("->" + newSynsetID);
						System.out.println("->" + (synsetURI = WordNetRDF.getSynsetFromID(newSynsetID)));
					}
					else if (synset.charAt(0) == 'R') {
						String newSynsetID = synset.replace('R', '4');
						System.out.print("->" + newSynsetID);
						System.out.println("->" + (synsetURI = WordNetRDF.getSynsetFromID(newSynsetID)));
					}
					else
						throw new Exception("unkown POS");
					
					if (synsetURI == null)
						throw new Exception();
					
					List<String> concepts = mappings.get( synsetURI );
					if (concepts == null) {
						concepts = new ArrayList<String>();
						mappings.put(synsetURI, concepts);
					}
					concepts.add( concept );
				}
			}
		}

		catch ( Exception e ) {
			e.printStackTrace();
		}
	}

}
