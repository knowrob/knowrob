package instruction.wordnet;

import java.util.HashMap;

/**
 * Maps prepositions to corresponding Cyc Concepts
 * 
 * @author Daniel Nyga
 * 
 */
public class PrepositionalMap {
	
	public static final String IN_FRONT_OF = "inFrontOf-Generally";
	public static final String IN = "in-UnderspecifiedContainer";
	public static final String ON = "on-UnderspecifiedSurface";
	public static final String NEXT_TO = "nextToLikeObjects";
	public static final String AT = "at-UnderspecifiedLandmark";
	public static final String OF = "parts-Underspecified";//"physicalParts";
	public static final String TO = "to-UnderspecifiedLocation";
	public static final String FROM = "from-UnderspecifiedLocation";
	public static final String THROUGH = "through-UnderspecifiedPortal";
	public static final String AMONG = "among-Underspecified";
	public static final String INSIDE = "inside-UnderspecifiedRegion";
	public static final String ABOUT = "about-UnderspecifiedRegion";
	public static final String WITHOUT = "without-Underspecified";
	public static final String ALONG = "along-UnderspecifiedPath";
	public static final String AROUND = "around-UnderspecifiedRegion";
	public static final String BY = "by-Underspecified";
	public static final String UNDER = "under-UnderspecifiedLocation";
	public static final String OVER = "over-UnderspecifiedLocation";
	public static final String WITH = "with-UnderspecifiedAgent";
	public static final String INTO = "into-UnderspecifiedContainer";
	public static final String ACROSS = "across-UnderspecifiedRegion";
	public static final String FOR = "for-UnderspecifiedLocation";
	public static final String AFTER = "after-Underspecified";
	public static final String BEFORE = "before-Underspecified";
	public static final String AGAINST = "against-Underspecified";

	/** The map: preposition -> Cyc Concept */
	private static HashMap<String, String> map = null;

	/**
	 * Returns the Cyc Constant that corresponds to the preposition <code>p</code>
	 * 
	 * @param p
	 * @return
	 */
	public static String get( String p ) {

		if ( map == null )
			init();
		return map.get( p.toLowerCase() );
	}

	/**
	 * Initialize the map - only for internal use
	 */
	private static void init() {

		map = new HashMap<String, String>();

		map.put( "in front of", IN_FRONT_OF );
		map.put( "in", IN );
		map.put( "into", INTO );
		map.put( "on", ON );
		map.put( "onto", ON );
		map.put( "next to", NEXT_TO );
		map.put( "at", AT );
		map.put( "of", OF );
		map.put( "to", TO );
		map.put( "from", FROM );
		map.put( "through", THROUGH);
		map.put( "among", AMONG );
		map.put( "inside", INSIDE );
		map.put( "about", ABOUT );
		map.put( "without", WITHOUT );
		map.put( "along", ALONG );
		map.put( "around", AROUND );
		map.put( "by", BY );
		map.put( "under", UNDER );
		map.put( "over", OVER );
		map.put( "with", WITH );
		map.put( "across", ACROSS );
		map.put( "for", FOR );
		map.put( "after", AFTER );
		map.put( "before", BEFORE );
		map.put( "against", AGAINST );
	}
}
