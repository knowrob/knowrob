package edu.tum.cs.ias.knowrob.vis.actions;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Vector;


import edu.tum.cs.ias.knowrob.owl.OWLClass;
import edu.tum.cs.ias.knowrob.owl.OWLThing;
import edu.tum.cs.ias.knowrob.owl.ObjectInstance;
import edu.tum.cs.ias.knowrob.prolog.PrologInterface;


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
	 * Remove action from sub-actions list (iterate deeper into sub-action hierarchy if needed)
	 * 
	 * @param sub-action sub-action to remove
	 */
	public void removeSubAction(Action sub_action) {
		
		// either remove on this level or iterate deeper until action is found
		if(sub_actions.contains(sub_action)) {
			sub_actions.remove(sub_action);
			
		} else {
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
		
		for(Action sub : getSubActions()) {
			res.addAll(sub.getTransitionsRecursive());
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
		
		
		// Read the action's label if an rdfs:label is set (assuming IRI has been set during initialization) 
		try {
			HashMap<String, Vector<String>> qLabel = PrologInterface.executeQuery("rdf_has('"+iri+"',rdfs:label,L),util:strip_literal_type(L,Label)");
			
			if(qLabel.get("Label")!=null && qLabel.get("Label").size()>0) {
				this.label = OWLThing.removeSingleQuotes(qLabel.get("Label").get(0));
			}			
		} catch (Exception e) { } // fail silently if no label is set


		// Read superclasses of action 
		HashMap<String, Vector<String>> qSuper = PrologInterface.executeQuery("owl_direct_subclass_of('" + iri + "', Super)");

		if(qSuper != null) {

			for(String sup : qSuper.get("Super")) {

				if (sup.contains("__Description"))
					continue;

				superclasses.add(OWLClass.getOWLClass(OWLThing.removeSingleQuotes(sup)));
			}
		}
		

		// Read action properties
		try {
			
			HashMap<String, Vector<String>> qProp = 
				PrologInterface.executeQuery("((class_properties_some('"+iri+"', Prop, V), Type='some'); " +
						  "(class_properties_all('"+iri+"', Prop, V), Type='all'); " +
						  "(class_properties_value('"+iri+"', Prop, V), Type='value')), " +
				   		  "util:strip_literal_type(V,Val)");
			
			if(qProp != null) {
				
				Vector<String> prop = qProp.get("Prop");
				Vector<String> val  = qProp.get("Val");
				Vector<String> type = qProp.get("Type");

				
				// Make sure each property is added only once 
				// (properties may be present two or more times in the result set)
				
				HashSet<String> alreadyAdded = new HashSet<String>();
				if(prop != null && val != null)
					
					for(int i=0;i<prop.size() && i<val.size();i++) {
						
						if (alreadyAdded.contains(prop.get(i)+val.get(i)))
							continue;
						
						alreadyAdded.add(prop.get(i)+val.get(i));
						String p = OWLThing.removeSingleQuotes(prop.get(i));
						String v = OWLThing.removeSingleQuotes(val.get(i));

						if((p.compareToIgnoreCase("subAction") != 0) && (p.compareToIgnoreCase("stateTransition") != 0)) {
							
							if(type.contains("some")) {
								this.addSomeValuesFrom(p, v);
								
							} else if(type.contains("all")) {
								this.addAllValuesFrom(p, v);
								
							} else if(type.contains("value")) {
								this.addHasValue(p, v);
							}
							
							
							if(p.endsWith("taskStartState")) {
								transitions.setStartAction(Action.getAction(v));
							}
						}
					}
			}

		} catch (Exception e) {
			e.printStackTrace();
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
	}
}
