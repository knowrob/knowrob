package edu.tum.cs.ias.knowrob.owl;

import java.util.Hashtable;
import java.util.Random;

public class OWLThing implements Comparable<OWLThing> {
	
	
	/**
	 * Map containing references to all existing OWL identifiers to avoid duplicate creation
	 */
	protected static Hashtable<String, OWLThing> identifiers = new Hashtable<String, OWLThing>();
	
	/**
	 * Unique identifier of this Thing
	 */
	protected String iri;
	
	/**
	 * Natural-language label
	 */
	protected String label;
	
	/**
	 * Flag indicating if the properties of this Thing have been read from Prolog 
	 */
	protected boolean readFromProlog = false;


	/**
	 * Flag indicating if local modifications need to be synchronized with Prolog
	 */
	protected boolean saveToProlog = false;
	
	
	
	/**
	 * OWLThing factory. Return existing instance, if available, and create new
	 * OWLThing instance if necessary. Avoids duplicate instances with the same 
	 * IRI.
	 * 
	 * @param iri Identifier of this thing.
	 * @param label Optional natural-language label.
	 * @return Instance of an {@link OWLThing} with the specified IRI
	 */
	public static OWLThing getOWLThing(String iri, String label) {
		
		if(identifiers.containsKey(iri)) {

			if(identifiers.get(iri).label != label)
				identifiers.get(iri).setLabel(label);
			
			return identifiers.get(iri);
			
		} else {
			OWLThing res = new OWLThing(iri, label);
			identifiers.put(iri, res);			
			return res;
		}
	}
	
	
	/**
	 * Constructor. Set the IRI and optionally a label. If none is given, 
	 * it is initialized with the IRI's short name.
	 *  
	 * @param iri Identifier of this thing.
	 * @param label Optional natural-language label.
	 */
	protected OWLThing(String iri, String label) {
		
		this.iri = iri;
		
		if(label!=null)
			this.label = label;
		else 
			this.label = getShortName();

	}
	
	
	/**
	 * Search the IRI map for IRIs that end with the given short
	 * name, return the object or null if not found
	 * 
	 * @param iri
	 * @param label
	 * @return
	 */
	public static OWLThing getOWLThingByShortname(String shortname) {
		
		for(String iri : identifiers.keySet()) {
			if(iri.endsWith(shortname)) {
				return identifiers.get(iri);
			}
		}
		return null;
	}

	/**
	 * Remove the IRI from the internal buffer.
	 * 
	 * @param iri IRI of the Thing to be removed
	 */
	public static void removeOWLThing(String iri) {
		identifiers.remove(iri);
	}
	
	
	/**
	 * Get the IRI of this {@link OWLThing}
	 * @return IRI of this {@link OWLThing}
	 */
	public String getIRI() {
		return iri;
	}

	
	/**
	 * Set the IRI of this {@link OWLThing}
	 * @param iri IRI of this {@link OWLThing}
	 */
	public void setIri(String iri) {
		this.iri = iri;
	}

	/**
	 * Get the Label of this {@link OWLThing}
	 * @return Label of this {@link OWLThing}
	 */
	public String getLabel() {
		return label;
	}
	
	/**
	 * Set the Label of this {@link OWLThing}
	 * @param label Label of this {@link OWLThing}
	 */
	public void setLabel(String label) {
		this.label = label;
	}
	
	
	/**
	 * Get the IRI prefix, i.e. the part of the IRI in front of the 
	 * hash sign ('#')
	 * 
	 * @return Prefix of the IRI
	 */
	public String getIRIPrefix(){
		return getPrefixOfIRI(iri);
	}



	/**
	 * Get the thing's short name, i.e. the part of the IRI after the 
	 * hash sign ('#')
	 * 
	 * @return Short name of the {@link OWLThing}
	 */
	public String getShortName(){ // name after hash sign
		return getShortNameOfIRI(iri);
	}
	
	
	/**
	 * Get the filename part of the IRI, the one after the last slash '/' 
	 * and before the hash sign ('#')
	 * 
	 * @return Filename part of the IRI
	 */
	public String getIRIFilename(){ // after last '/', before hash
		return getFilenameOfIRI(iri);
	}
	


	/**
	 * Implement the {@link Comparable} interface. {@link OWLThing} are compared 
	 * based on their IRI.
	 * 
	 * @param o {@link OWLThing} to compare to
	 */
	@Override
	public int compareTo(OWLThing o) {
		return this.iri.compareTo(o.getIRI());
	}
	
	
	/**
	 * Equality of OWL classes is defined as equality of their IRIs
	 * 
	 * @param b {@link OWLThing} to compare to
	 * @return true if the IRIs are equal
	 */
	public boolean equals(Object b) {
		if (b instanceof OWLThing) {
			if(this.iri.equals(((OWLThing) b).getIRI()))
				return true;
			else
				return false;
		}
		return false;
	}
	
	/**
	 * Print as the IRI
	 */
	@Override
	public String toString() {
		return this.iri;
	}
	
	
    /**
     * Removes single quotes at the start and end of a string if applicable. 
     * 
     * Useful when dealing with OWL IRI identifiers that need to be 
     * in single quotes in Prolog. 
     * 
     * @param str String with or without single quotes at the beginning and end
     * @return String without single quotes at the beginning and end
     */
    public static String removeSingleQuotes(String str) {
        if(str.startsWith("'"))
            str = str.substring(1);
        
        if(str.endsWith("'"))
            str = str.substring(0, str.length()-1);
        return str;
    }
    

    /**
     * Adds single quotes at the start and end of a string if applicable. 
     * 
     * Useful when dealing with OWL IRI identifiers that need to be 
     * in single quotes in Prolog. 
     * 
     * @param str String with or without single quotes at the beginning and end
     * @return String with single quotes at the beginning and end
     */
    public static String addSingleQuotes(String str) {
        return "'"+removeSingleQuotes(str)+"'";
    }
    

	/**
	 * Get the short name of the provided IRI, i.e. the part of the IRI after the 
	 * hash sign ('#')
	 * 
	 * @return Short name of the IRI
	 */
    public static String getShortNameOfIRI(String iri) {

    	String[] elem = iri.split("#");
    	if(elem.length>1) {
    		String res = elem[1].replaceAll("'", "");
    		return res;
    	}
    	else return iri;
    }
	

	public static String getPrefixOfIRI(String iri) {
		
        String[] elem = iri.split("#");
        if(elem.length>1) {
            String res = elem[0].replaceAll("'", "");
            return res;
        }
        else return iri;
	}

	public static String getFilenameOfIRI(String iri) {
		
        String[] elem = iri.split("/");
        if(elem.length>1) {
        	
            String after_slash = elem[elem.length-1].replaceAll("'", "");
            String[] elem2 = after_slash.split("#");
            
            if(elem2.length>1) {
                String res = elem2[0];
                return res;
            }
            else return after_slash;
        }
        else return iri;
	}

	/**
	 * Generate unique identifiers by adding a quasi-unique suffix to an IRI.
	 * 
	 * @param iri Base IRI to be extended
	 * @return Concatenation of iri with unique suffix
	 */
	public static String getUniqueID(String iri) {
		
		char[] chars = "abcdefghijklmnopqrstuvwxyzABSDEFGHIJKLMNOPQRSTUVWXYZ1234567890".toCharArray();
		Random r = new Random(System.currentTimeMillis());
		char[] id = new char[8];
		for (int i = 0;  i < 8;  i++) {
		    id[i] = chars[r.nextInt(chars.length)];
		}
		try { Thread.sleep(5); } catch (InterruptedException e) {
			e.printStackTrace();
		}
		return iri + "_" + new String(id);
	} 
	
	public boolean isReadFromProlog() {
		return readFromProlog;
	}


	public void setReadFromProlog(boolean readFromProlog) {
		this.readFromProlog = readFromProlog;
	}


	public boolean needsSaveToProlog() {
		return saveToProlog;
	}


	public void setSaveToProlog(boolean saveToProlog) {
		this.saveToProlog = saveToProlog;
	}
	
}
