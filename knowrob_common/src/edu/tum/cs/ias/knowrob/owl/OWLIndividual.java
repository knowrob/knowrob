package edu.tum.cs.ias.knowrob.owl;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Vector;


public class OWLIndividual extends OWLThing {
	

	/**
	 * List of strings representing the object types (OWL classes)
	 */
	protected Vector<OWLClass> types;
	
	/**
	 * Map of properties that are defined for this individual to their respective values.
	 */
	protected Map<String, Vector<String>> properties;
	


	/**
	 * Constructor. Set the IRI and optionally a label. If none is given, 
	 * it is initialized with the IRI's short name.
	 * 
	 * @param iri Identifier of this thing.
	 * @param label Optional natural-language label.
	 */
	protected OWLIndividual(String iri, String label) {
		
		super(iri, label);
		this.types = new Vector<OWLClass>();
		properties = Collections.synchronizedMap(new HashMap<String, Vector<String>>());
	}


	/**
	 * Copy constructor: create OWLIndividual from more generic {@link OWLThing}
	 * 
	 * @param ind {@link OWLIndividual} to be copied into this {@link OWLThing}
	 */
	protected OWLIndividual(OWLThing ind) {
		this(ind.getIRI(), ind.getLabel());
	}
	
	
	/**
	 * OWLIndividual factory. Return existing instance, if available, and create new
	 * OWLIndividual instance if necessary. Avoids duplicate instances with the same 
	 * IRI.
	 * 
	 * @param iri Identifier of this thing.
	 * @param label Optional natural-language label.
	 * @return Instance of an {@link OWLIndividual} with the specified IRI
	 */
	public static OWLIndividual getOWLIndividual(String iri, String label) {

		// return exact match if available
		if(identifiers.containsKey(iri) && identifiers.get(iri) instanceof OWLIndividual) {
			return (OWLIndividual) identifiers.get(iri);			
		}
		
		// create OWLIndividual from higher-level objects if the existing object for this IRI has a more abstract type
		OWLIndividual res = new OWLIndividual(OWLThing.getOWLThing(iri, label));
		identifiers.put(iri, res);
		return res;
	}
	
	/**
	 * OWLIndividual factory. Return existing instance, if available, and create new
	 * OWLIndividual instance if necessary. Avoids duplicate instances with the same 
	 * IRI.
	 * 
	 * @param iri Identifier of this thing.
	 * @return Instance of an {@link OWLIndividual} with the specified IRI
	 */
	public static OWLIndividual getOWLIndividual(String iri) {
		return getOWLIndividual(iri, null); 
	}
	
	
	

	/**
	 * Get 
	 * @return
	 */
	public Vector<OWLClass> getTypes() {
		return types;
	}

	/**
	 * 
	 * @param types
	 */
	public void setTypes(Vector<OWLClass> types) {
		this.types.clear();
		this.types.addAll(types);
	}

	/**
	 * 
	 * @param type
	 */
	public void addType(OWLClass type) {
		types.add(type);
	}

	/**
	 * 
	 * @param type
	 */
	public void addTypes(Collection<OWLClass> t) {
		types.addAll(t);
	}

	/**
	 * 
	 * @param type
	 * @return
	 */
	public boolean hasType(OWLClass type) {
		
		if(types.contains(type))
			return true;
		else
			return false;
	}
	
	public boolean hasType(String type_iri) {
		
		for(OWLClass c : types) {
			if(c.getIRI().equals(type_iri) || c.getShortName().equals(type_iri))
				return true;
		}
		return false;
	}
	
	/**
	 * Get the properties defined for this individual.
	 * 
	 * @return Map from property identifiers to a Vector with their values.
	 */
	public Map<String, Vector<String>> getProperties() {
		return properties;
	}
	
	/**
	 * Get the values for a specific property.
	 * 
	 * @return Vector of strings referring to the property values
	 */
	public Vector<String> getProperty(String prop) {
		return properties.get(prop);
	}
	
	/**
	 * Set the internal properties map to the given one (after resetting it first).
	 * 
	 * @param properties New map of properties to be used.
	 */
	public void setProperties(Map<String, Vector<String>> properties) {
		this.properties.clear();
		this.properties.putAll(properties);
	}
	
	
	/**
	 * Check whether a property is defined for this individual.
	 * 
	 * @param property Identifier of the property to be checked.
	 * @return true if this property exists
	 */
	public boolean hasProperty(String property) {
		if(properties.containsKey(property))
			return true;
		else return false;
	}
	
	/**
	 * Get a {@link Vector} of all values defined for a property.
	 *  
	 * @param property Identifier of the property to be retrieved.
	 * @return Vector with the defined values.
	 */
	public Vector<String> getPropValues(String property) {
		return properties.get(property);
	}
	
}
