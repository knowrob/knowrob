package edu.tum.cs.ias.knowrob.vis.actions;

import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Vector;


import edu.tum.cs.ias.knowrob.owl.OWLClass;
import edu.tum.cs.ias.knowrob.owl.ObjectInstance;


public class Action extends OWLClass{

	/**
	 * Sequence of sub-actions of this action
	 */
	private LinkedList<Action> sub_actions;
	
	/**
	 * Parent action if this action is in a subsequence
	 */
	private Action parentOfSequence = null;
	
	/**
	 * reference to currently expanded sequence
	 */
	private Action expandedSequence = null;


	/**
	 * Holds information for drawing this action.
	 * Used to avoid recalculation of position/size for each repainting
	 */
	private ActionDrawInformation drawInfo;
	
	
	

	/**
	 * Constructor. Set the IRI and optionally a label. If none is given, 
	 * it is initialized with the IRI's short name.
	 * 
	 * @param iri Identifier of this thing.
	 */
	protected Action(String iri, String label) {
		
		super(iri, label);
		this.drawInfo = new ActionDrawInformation(this);
		this.sub_actions = new LinkedList<Action>();

	}


	/**
	 * Copy constructor: create Action from more generic {@link OWLClass}
	 * 
	 * @param ind {@link OWLClass} to be copied into this {@link ObjectInstance}
	 */
	protected Action(OWLClass ind) {
		
		this(ind.getIRI(), ind.getLabel());
		
		// copy properties
		some_values_from.putAll(ind.getSomeValuesFrom());
		all_values_from.putAll(ind.getAllValuesFrom());
		has_value.putAll(ind.getHasValue());
	}
	
	
	/**
	 * Action factory. Return existing instance, if available, and create new
	 * Action instance if necessary. Avoids duplicate instances with the same 
	 * IRI.
	 * 
	 * @param iri Identifier of this thing.
	 * @param label Optional natural-language label.
	 * @return Instance of an {@link Action} with the specified IRI
	 */
	public static Action getAction(String iri, String label) {

		// return exact match if available
		if(identifiers.containsKey(iri) && identifiers.get(iri) instanceof Action) {
			return (Action) identifiers.get(iri);			
		}
		
		// create ObjectInstance from higher-level objects if the existing object for this IRI has a more abstract type
		Action res = new Action(OWLClass.getOWLClass(iri, label));
		identifiers.put(iri, res);
		return res;
	}
	
	/**
	 * Action factory. Return existing instance, if available, and create new
	 * Action instance if necessary. Avoids duplicate instances with the same 
	 * IRI.
	 * 
	 * @param iri Identifier of this thing.
	 * @return Instance of an {@link Action} with the specified IRI
	 */
	public static Action getAction(String iri) {
		return getAction(iri, null); 
	}


	
	
	/**
	 * Get map of all defined action properties. Convenience method combining values
	 * defined as some_values_from and has_value restrictions.
	 * 
	 * The calue is an array of strings because a property may have multiple values.
	 * 
	 * @return list of action properties
	 */
	public Map<String, Vector<String>> getProperties() {
		
		Map<String, Vector<String>> res = new LinkedHashMap<String, Vector<String>>();
		
		res.putAll(some_values_from);
		res.putAll(has_value);
		
		// also add superclasses
		if(getSuperClasses()!=null && getSuperClasses().size() > 0) {
			res.put("type", new Vector<String>());
			for(OWLClass cl : getSuperClasses()) {
				res.get("type").add(cl.getIRI());
			}
		}
		
		return res;
	}
	
	/**
	 * Get property with specified key. Convenience method combining values
	 * defined as some_values_from and has_value restrictions.
	 * 
	 * @param key Key of property
	 * @return Value of property or null if not found
	 */
	public List<String> getProperty(String key) {
		
		if(some_values_from.containsKey(key)){
			return some_values_from.get(key);	
		} else if(has_value.containsKey(key)) {
			return has_value.get(key);
		} else if (key.equals("type")){
			
			Vector<String> res = new Vector<String>();
			for(OWLClass cl : superclasses) {
				res.add(cl.getLabel());
			}
			return res;
		} else {
			return null;
		}
		
	}
	


	/**
	 * Get information for drawing the action
	 * @return object containing draw information
	 */
	public ActionDrawInformation getDrawInfo() {
		return drawInfo;
	}
	

	

	/**
	 * Get iterator for sub-actions.
	 * Use the provided dedicated methods to add or remove a sub-action.
	 * 
	 * @return Iterator for sub-actions list
	 */
	public Iterator<Action> getSubActionsIterator() {
		return sub_actions.iterator();
	}

	/**
	 * Returns the number of sub-actions for this action
	 * 
	 * @return length of sub-actions iterator
	 */
	public int getSubActionsCount() {
		return sub_actions.size();
	}
	
	/**
	 * Add an action to the sub-actions listfor this action.
	 * 
	 * @param sub_action action to add
	 */
	public void addSubAction(Action sub_action) {
		this.sub_actions.add(sub_action);
		sub_action.parentOfSequence = this;
		drawInfo.notifyModified();
	}
	
	/**
	 * Add multiple actions to the sub-actions list for this action.
	 * 
	 * @param sub_actions List of actions to add to sequence list
	 */
	public void addSubActions(LinkedList<Action> sub_actions) {
		
		this.sub_actions.addAll(sub_actions);
		
		for (Action s : sub_actions) {
			s.parentOfSequence = this;
		}
		
		drawInfo.notifyModified();
	}
	

	/**
	 * Remove action from sub-actions list
	 * 
	 * @param sub-action sub-action to remove
	 */
	public void removeSubAction(Action sub_action) {
		this.sub_actions.remove(sub_action);
		drawInfo.notifyModified();
	}
	
	
	
	
	/**
	 * Returns true if the sequence is expanded.
	 * @return true if expanded
	 */
	public boolean isExpanded() {
		if(parentOfSequence!=null)
			return (parentOfSequence.expandedSequence == this);
		else 
			return false;
	}
	
	/**
	 * Get the currently expanded subsequence
	 * @return Action which is currently expanded
	 */
	public Action getExpandedSequence() {
		return expandedSequence;
	}
	
	/**
	 * Expand the given action in this actions subsequence
	 * @param sequence Subcation to expand
	 */
	public void setExpandedSequence(Action sequence) {
		if (this.sub_actions.contains(sequence))
		{
			this.expandedSequence = sequence;
			if (parentOfSequence != null)
				parentOfSequence.drawInfo.notifyModified();
		}
	}
	
	/**
	 * Toggle the expanded value
	 */
	public void toggleExpand()
	{
		if (parentOfSequence.expandedSequence == this)
			parentOfSequence.expandedSequence = null;
		else
			parentOfSequence.expandedSequence = this;
			parentOfSequence.drawInfo.notifyModified();
	}
}
