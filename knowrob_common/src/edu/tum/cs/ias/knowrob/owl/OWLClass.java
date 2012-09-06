package edu.tum.cs.ias.knowrob.owl;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Vector;

public class OWLClass extends OWLThing {

	
	/**
	 * Vector of subclasses
	 */
	protected Vector<OWLClass> subclasses;
	
	/**
	 * Reference to the parent / superclass
	 */
	protected Vector<OWLClass> superclasses;


	/**
	 * Map of existential restrictions (someValuesFrom)
	 */
	protected Map<String, Vector<String>> some_values_from;

	/**
	 * Map of universal restrictions (allValuesFrom)
	 */
	protected Map<String, Vector<String>> all_values_from;

	/**
	 * Map of value restrictions (hasValue)
	 */
	protected Map<String, Vector<String>> has_value;
	
	
	
	
	/**
	 * Constructor. Set the IRI and optionally a label. If none is given, 
	 * it is initialized with the IRI's short name.
	 * 
	 * @param iri Identifier of this thing.
	 * @param label Optional natural-language label.
	 */
	protected OWLClass(String iri, String label) {
		
		super(iri, label);
		this.superclasses = new Vector<OWLClass>();
		this.subclasses = new Vector<OWLClass>();
		
		some_values_from = Collections.synchronizedMap(new LinkedHashMap<String, Vector<String>>());
		all_values_from = Collections.synchronizedMap(new LinkedHashMap<String, Vector<String>>());
		has_value = Collections.synchronizedMap(new LinkedHashMap<String, Vector<String>>());
	}


	/**
	 * Copy constructor: create OWLClass from more generic {@link OWLThing}
	 * 
	 * @param ind {@link OWLClass} to be copied into this {@link OWLThing}
	 */
	protected OWLClass(OWLThing ind) {
		this(ind.getIRI(), ind.getLabel());
	}
	
	
	/**
	 * OWLClass factory. Return existing instance, if available, and create new
	 * OWLClass instance if necessary. Avoids duplicate instances with the same 
	 * IRI.
	 * 
	 * @param iri Identifier of this thing.
	 * @param label Optional natural-language label.
	 * @return Instance of an {@link OWLClass} with the specified IRI
	 */
	public static OWLClass getOWLClass(String iri, String label) {

		// return exact match if available
		if(identifiers.containsKey(iri) && identifiers.get(iri) instanceof OWLClass) {
			return (OWLClass) identifiers.get(iri);			
		}
		
		OWLClass res = new OWLClass(OWLThing.getOWLThing(iri, label));
		identifiers.put(iri, res);
		return res;
	}
	
	/**
	 * OWLClass factory. Return existing instance, if available, and create new
	 * OWLClass instance if necessary. Avoids duplicate instances with the same 
	 * IRI.
	 * 
	 * @param iri Identifier of this thing.
	 * @return Instance of an {@link OWLClass} with the specified IRI
	 */
	public static OWLClass getOWLClass(String iri) {
		return getOWLClass(iri, null); 
	}
	
	
	
	
	/**
	 * Get all subclasses defined for this OWL class.
	 * 
	 * @return Vector of subclasses
	 */
	public Vector<OWLClass> getSubclasses() {
		return subclasses;
	}
	
	
	/**
	 * Add a subclass definition. Complementary to addParentClass.
	 * 
	 * @param sub Subclass
	 */
	public void addSubclass(OWLClass sub) {
		subclasses.add(sub);
	}
	
	
	/**
	 * Remove a subclass from this class
	 * 
	 * @param sub Subclass to be removed
	 */
	public void removeSubclass(OWLClass sub) {
		subclasses.remove(sub);
	}
	
	
	
	/**
	 * Set the superclasses of this class
	 * 
	 * @param p Parent class
	 */
	public void setSuperClasses(Vector<OWLClass> p) {
		this.superclasses.clear();
		this.superclasses.addAll(p);
	}
	
	/**
	 * Add a superclass definition. Complementary to addSubclass.
	 * 
	 * @param p Superclass
	 */
	public void addSuperClass(OWLClass p) {
		superclasses.add(p);
	}
	
	
	/**
	 * Get the superclasses of this class
	 * @return
	 */
	public Vector<OWLClass> getSuperClasses() {
		return superclasses;
	}
	
	
	
	
	/**
	 * Set the someValuesFrom restrictions
	 * 
	 * @param someValuesFrom Map with the defined restrictions
	 */
	public void setSomeValuesFrom(Map<String, Vector<String>> someValuesFrom) {
		this.some_values_from.clear();
		this.some_values_from.putAll(someValuesFrom);
	}
	
	
	/**
	 * Add a someValuesFrom  definition
	 * 
	 * @param prop Property to be used in restriction
	 * @param classdef Class definition for restriction
	 */
	public void addSomeValuesFrom(String prop, String classdef) {
		
		if(!some_values_from.containsKey(prop)) {
			some_values_from.put(prop, new Vector<String>());
		}
		some_values_from.get(prop).add(classdef);
	}
	
	
	/**
	 * Get the someValuesFrom definitions of this class
	 * @return someValuesFrom definitions
	 */
	public Map<String, Vector<String>> getSomeValuesFrom() {
		return some_values_from;
	}
	
	
	
	
	/**
	 * Set the allValuesFrom restrictions
	 * 
	 * @param allValuesFrom Map with the defined restrictions
	 */
	public void setAllValuesFrom(Map<String, Vector<String>> allValuesFrom) {
		this.all_values_from.clear();
		this.all_values_from.putAll(allValuesFrom);
	}
	
	
	/**
	 * Add a allValuesFrom  definition
	 * 
	 * @param prop Property to be used in restriction
	 * @param classdef Class definition for restriction
	 */
	public void addAllValuesFrom(String prop, String classdef) {
		
		if(!all_values_from.containsKey(prop)) {
			all_values_from.put(prop, new Vector<String>());
		}
		all_values_from.get(prop).add(classdef);
	}
	
	
	/**
	 * Get the allValuesFrom definitions of this class
	 * 
	 * @return allValuesFrom definitions
	 */
	public Map<String, Vector<String>> getAllValuesFrom() {
		return all_values_from;
	}
	
	
	
	
	/**
	 * Set the hasValue restrictions
	 * 
	 * @param hasValue Map with the defined restrictions
	 */
	public void setHasValue(Map<String, Vector<String>> hasValue) {
		this.has_value.clear();
		this.has_value.putAll(hasValue);
	}
	
	
	/**
	 * Add a hasValue  definition
	 * 
	 * @param prop Property to be used in restriction
	 * @param value Individuals or values to be used for the restriction
	 */
	public void addHasValue(String prop, String value) {
		
		if(!has_value.containsKey(prop)) {
			has_value.put(prop, new Vector<String>());
		}
		has_value.get(prop).add(value);
	}
	
	
	/**
	 * Get the hasValue definitions of this class
	 * @return hasValue definitions
	 */
	public Map<String, Vector<String>> getHasValue() {
		return has_value;
	}
	
	
}
