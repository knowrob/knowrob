package instruction.opencyc;

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
import org.jdom.JDOMException;
import org.jdom.input.SAXBuilder;
import org.jdom.output.XMLOutputter;

public class CycCache {

	public static final String TAG_CACHE = "cyccache";

	public static final String TAG_SYNSET = "synsetid";

	public static final String TAG_CONCEPT = "cycconcept";

	public static final String ATTR_VALUE = "value";

	Map<String, List<String>> map = new HashMap<String, List<String>>();

	/**
	 * Puts a new list of Cyc concept names to be associated with the Synset-ID
	 * <code>synset</code> to the cache
	 * 
	 * @param synset
	 * @param concepts
	 */
	public void tell( String synset, List<String> concepts ) {

		map.put( synset, concepts );
	}

	/**
	 * Checks if the Synset-ID <code>synset</code> already has been cached
	 * 
	 * @param synset
	 * @return <code>null</code> if the Synset-ID has not been found in the
	 *         cache
	 */
	public List<String> ask( String synset ) {

		return map.get( synset );
	}

	/**
	 * Loads the cache file <code>etc/cyccache.xml</code>
	 * 
	 * @throws IOException
	 * @throws
	 * @throws Exception
	 */
	@SuppressWarnings("unchecked")
	public void load() throws IOException {

		File file = new File( "etc/cyccache.xml" );
		if ( ! file.exists() )
			return;

		FileReader reader = new FileReader( file );

		SAXBuilder sax = new SAXBuilder();
		try {
			Document doc;

			doc = sax.build( reader );

			Element cache = doc.getRootElement();

			List<Element> synsetids = cache.getChildren();

			// ==================================================================
			// Iterate over Synset-IDs
			// ==================================================================
			for ( Iterator<Element> i = synsetids.iterator(); i.hasNext(); ) {

				Element synsetid = i.next();
				String id = synsetid.getAttributeValue( ATTR_VALUE );
				if ( id == null )
					continue;

				List<String> conceptNames = new ArrayList<String>();
				List<Element> concepts = synsetid.getChildren();

				// ==================================================================
				// Iterate over associated Cyc concept names
				// ==================================================================
				for ( Iterator<Element> j = concepts.iterator(); j.hasNext(); ) {
					Element concept = j.next();
					String conceptName = concept.getAttributeValue( ATTR_VALUE );
					if ( conceptName == null )
						continue;
					conceptNames.add( conceptName );

				}

				map.put( id, conceptNames );

			}
		}
		catch ( JDOMException e ) {
			e.printStackTrace();
		}

	}

	/**
	 * Saves the cache to the file <code>etc/cyccache.xml</code>
	 * 
	 * @throws Exception
	 */
	public void save() throws Exception {

		File file = new File( "etc/cyccache.xml" );
		if ( ! file.exists() )
			file.createNewFile();

		Element cache = new Element( TAG_CACHE );

		Set<String> synsets = map.keySet();

		// ==================================================================
		// Iterate over all keys (Synset-IDs) contained in the cache
		// ==================================================================
		for ( Iterator<String> i = synsets.iterator(); i.hasNext(); ) {
			Element synset = new Element( TAG_SYNSET );
			String synsetid = i.next();
			synset.setAttribute( ATTR_VALUE, synsetid );

			// ==================================================================
			// Iterate over all Cyc concept names associated with the current
			// Synset-ID
			// ==================================================================
			List<String> concepts = map.get( synsetid );

			for ( Iterator<String> j = concepts.iterator(); j.hasNext(); ) {
				String conceptName = j.next();
				Element concept = new Element( TAG_CONCEPT );
				concept.setAttribute( ATTR_VALUE, conceptName );
				synset.addContent( concept );
			}
			cache.addContent( synset );
		}

		// ==================================================================
		// Create XML document and write it to file
		// ==================================================================
		Document doc = new Document( cache );

		XMLOutputter outPutter = new XMLOutputter( " ", true );
		FileWriter outStream = new FileWriter( file );

		outPutter.output( doc, outStream );
	}

}
