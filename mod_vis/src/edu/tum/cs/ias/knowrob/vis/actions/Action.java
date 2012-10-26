package edu.tum.cs.ias.knowrob.vis.actions;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Vector;


import edu.tum.cs.ias.knowrob.owl.OWLClass;
import edu.tum.cs.ias.knowrob.owl.ObjectInstance;
import edu.tum.cs.ias.knowrob.prolog.PrologInterface;
import edu.tum.cs.ias.knowrob.prolog.PrologQueryUtils;


public class Action extends OWLClass {

	/**
	 * Sub-actions of this action
	 */
	private List<Action> sub_actions;
	
	/**
	 * Transitions from this action
	 */
	private ActionTransitions transitions;
	
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
		this.sub_actions = Collections.synchronizedList(new ArrayList<Action>());
		this.transitions = new ActionTransitions();

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

		// also add superclasses
		if(getSuperClasses()!=null && getSuperClasses().size() > 0) {
			res.put("type", new Vector<String>());
			for(OWLClass cl : getSuperClasses()) {
				res.get("type").add(cl.getIRI());
			}
		}
		
		res.putAll(some_values_from);
		res.putAll(has_value);
		
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
	 * Returns the number of sub-actions for this action
	 * 
	 * @return length of sub-actions iterator
	 */
	public List<Action> getSubActions() {
		return sub_actions;
	}
	
	public List<Action> getSubActionsRecursive() {
		
		List<Action> res = new ArrayList<Action>();
		res.addAll(getSubActions());
		
		for(Action sub : getSubActions()) {
			res.addAll(sub.getSubActionsRecursive());
		}
		return res;
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
		
		synchronized(sub_actions) {
			this.sub_actions.add(sub_action);
		}
		sub_action.parentOfSequence = this;
		drawInfo.notifyModified();
	}
	
	/**
	 * Add multiple actions to the sub-actions list for this action.
	 * 
	 * @param sub_actions List of actions to add to sequence list
	 */
	public void addSubActions(LinkedList<Action> sub_actions) {
		
		synchronized(sub_actions) {
			this.sub_actions.addAll(sub_actions);
		}
		
		for (Action s : sub_actions) {
			s.parentOfSequence = this;
		}
		
		drawInfo.notifyModified();
	}
	

	/**
	 * Remove action from sub-actions list (iterate deeper into sub-action hierarchy if needed)
	 * 
	 * @param sub-action sub-action to remove
	 */
	public void removeSubAction(Action sub_action) {
		
		// either remove on this level or iterate deeper until action is found
		if(sub_actions.contains(sub_action)) {
			
			sub_actions.remove(sub_action);
			
			// adapt task start state and end state if the respective action has been deleted
			if(has_value.containsKey("http://ias.cs.tum.edu/kb/knowrob.owl#taskStartState") &&
			   has_value.get("http://ias.cs.tum.edu/kb/knowrob.owl#taskStartState").contains(sub_action.getIRI())) {
				
				// remove old start state definition
				has_value.get("http://ias.cs.tum.edu/kb/knowrob.owl#taskStartState").remove(sub_action.getIRI());
				
				if(sub_action.getTransitions()!=null && 
						sub_action.getTransitions().getTransitionsFrom(sub_action)!=null) {
					
					Set<ActionTransition> trans = sub_action.getTransitions().getTransitionsFrom(sub_action);
					
					// add the first to-action of the transitions from the deleted action as new start state
					for(ActionTransition t : trans) {
						has_value.get("http://ias.cs.tum.edu/kb/knowrob.owl#taskStartState").add(t.to.getIRI());
						break;
					}
				}
			}
			
			if(has_value.containsKey("http://ias.cs.tum.edu/kb/knowrob.owl#taskEndState") &&
			   has_value.get("http://ias.cs.tum.edu/kb/knowrob.owl#taskEndState").contains(sub_action.getIRI())) {

				// remove old start state definition
				has_value.get("http://ias.cs.tum.edu/kb/knowrob.owl#taskEndState").remove(sub_action.getIRI());
				
				// create new end state definitions
				if(getTransitionsRecursive()!=null && 
						getTransitionsRecursive().getTransitionsTo(sub_action)!=null) {
					
					Set<ActionTransition> trans = getTransitionsRecursive().getTransitionsTo(sub_action);
					
					// add all from-actions of the transitions from the deleted action as new end states
					for(ActionTransition t : trans) {
						has_value.get("http://ias.cs.tum.edu/kb/knowrob.owl#taskEndState").add(t.from.getIRI());
					}
				}
			}
			
		} else { // iterate deeper if a sub-action of a sub-action has been deleted
			
			for(Action sub : sub_actions) {
				sub.removeSubAction(sub_action);
			}
		}
		
		drawInfo.notifyModified();
	}
	

	public void addTransition(ActionTransition t) {
		this.transitions.add(t);
	}
	
	public void removeTransition(ActionTransition t) {
		this.transitions.remove(t);
	}
	
	public ActionTransitions getTransitions() {
		return transitions;
	}
	

	public ActionTransitions getTransitionsRecursive() {
		
		ActionTransitions res = new ActionTransitions();
		
		res.addAll(transitions);
		
		synchronized (sub_actions) {
			for(Action sub : sub_actions) {
				res.addAll(sub.getTransitionsRecursive());
			}
		}
		res.startAction = transitions.startAction;
		return res;
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
	
	

	/**
	 * Recursively read all properties of this action and all its subactions from Prolog.
	 */
	public void readFromProlog() {	
		
		if(isReadFromProlog())
			return;
		
		// read generic OWLClass properties 
		super.readFromProlog();

		// remove subAction and stateTransition properties from the generic property maps
		some_values_from.remove("http://ias.cs.tum.edu/kb/knowrob.owl#subAction");
		has_value.remove("http://ias.cs.tum.edu/kb/knowrob.owl#stateTransition");
		
		// post-process action-specific properties
		
		// set start state
		if(has_value.containsKey("http://ias.cs.tum.edu/kb/knowrob.owl#taskStartState")) {
			transitions.setStartAction(Action.getAction(has_value.get("http://ias.cs.tum.edu/kb/knowrob.owl#taskStartState").firstElement()));
		}

		// Read sub-actions of the current action
		try {
			
			HashMap<String, Vector<String>> qSub = PrologInterface.executeQuery("plan_subevents('"+iri+"',List)");

			if(qSub!=null) {

				Vector<String> list = qSub.get("List");

				//Add each action only once
				HashSet<String> alreadyAdded = new HashSet<String>();

				if(list != null) {
					
					for(String o : list) {
						for (String sArr[] : PrologInterface.dottedPairsToArrayList(o)) {
							for (String s : sArr) {
								
								s = PrologInterface.removeSingleQuotes(s);
								
								if (alreadyAdded.contains(s))
									continue;
								alreadyAdded.add(s);
								
								Action sub = Action.getAction(s);
								sub.readFromProlog();
								this.addSubAction(sub);
							}
						}
					}
				}
			}
			
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		
		
		// read action transitions for sub-actions
		try {
			
			HashMap<String, Vector<String>> qTrans = 
				PrologInterface.executeQuery("class_properties_value('"+iri+"', 'http://ias.cs.tum.edu/kb/knowrob.owl#stateTransition', Trans)");

			if(qTrans!=null && qTrans.get("Trans").size()>0) {

				for(String t : qTrans.get("Trans")) {

					t = PrologInterface.removeSingleQuotes(t);

					ActionTransition trans = ActionTransition.getActionTransition(t);
					trans.readFromProlog();
					trans.getFrom().addTransition(trans);
				}
			}
			
		} catch (Exception e) {
			e.printStackTrace();
		}

		// set flag that this action has been read
		this.setReadFromProlog(true);
	}
	
		

	/**
	 * Recursively write all properties of this action and all its sub-actions to Prolog.
	 */
	public void writeToProlog() {

		if(this.needsSaveToProlog()) {

			// remove old transitions 
			// IMPORTANT: this order because otherwise state transitions are not found since the superclass removes all of them
			PrologQueryUtils.clearActionStateTransitions(iri);
			
			// write all OWLClass-generic information to Prolog
			// (also sets flag that this action has been written)
			super.writeToProlog();

			// write transitions defined for sub-actions of this action
			for(Action sub : sub_actions) {
				for(ActionTransition t : sub.getTransitions()) {

					// write transitions as restrictions on this class
					String trans = PrologQueryUtils.createStateTransition(t.from.getIRI(), t.to.getIRI(), t.getCause());
					PrologQueryUtils.createRestriction(iri, "http://ias.cs.tum.edu/kb/knowrob.owl#stateTransition", trans, "http://www.w3.org/2002/07/owl#hasValue", "knowrob_java");
				}
			}
			
			this.setSaveToProlog(false);
		}
		
		// iterate over sub-actions to see if they need to be written
		for(Action sub : sub_actions) {
			sub.writeToProlog();

			// assert subAction relation
			PrologQueryUtils.createRestriction(iri, "http://ias.cs.tum.edu/kb/knowrob.owl#subAction", sub.getIRI(), "http://www.w3.org/2002/07/owl#someValuesFrom", "knowrob_java");
		}
	}
}
