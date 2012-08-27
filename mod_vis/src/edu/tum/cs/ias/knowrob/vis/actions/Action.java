package edu.tum.cs.ias.knowrob.vis.actions;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.LinkedHashMap;
import java.util.List;

/**
 * A action for a plan.
 * An action can have dependencies (parents) which have to be executed before.
 * An action may also have child actions which can be executed after this action.
 * 
 * Each action may also be decomposed into subsequences which are actions.
 * @author Stefan Profanter
 *
 */
public class Action {

	/**
	 * Properties of this action
	 * A key may have multiple values. So the type of value is an array of strings
	 */
	private LinkedHashMap<String, List<String>> properties;
	
	/**
	 * Parent actions in hierarchy
	 */
	private LinkedList<Action> parentActions;
	/**
	 * Child actions in hierarchy
	 */
	private LinkedList<Action> childActions;
	
	/**
	 * Subsequence of this action
	 */
	private LinkedList<Action> sequence;
	
	/**
	 * Parent action if this action is in a subsequence
	 */
	private Action parentOfSequence = null;
	
	/**
	 * reference to currently expanded sequence
	 */
	private Action expandedSequence = null;

	/**
	 * Name of the action
	 */
	private String name;
	

	/**
	 * Identifier of the action (from prolog)
	 */
	private String identifier;

	/**
	 * Holds information for drawing this action.
	 * Used to avoid recalculation of position/size for each repainting
	 */
	private ActionDrawInformation drawInfo;
	
	/**
	 * Constructor for initializing an action
	 * @param name Name/Title of the action
	 */
	public Action(String name) {
		this.properties = new LinkedHashMap<String, List<String>>();
		this.parentActions = new LinkedList<Action>();
		this.childActions = new LinkedList<Action>();
		this.sequence = new LinkedList<Action>();
		this.name = name;
		this.drawInfo = new ActionDrawInformation(this);
	}
	
	/**
	 * Constructor for initializing an action
	 * @param name Name/Title of the action
	 */
	public Action(String name, String identifier) {
		this.properties = new LinkedHashMap<String, List<String>>();
		this.parentActions = new LinkedList<Action>();
		this.childActions = new LinkedList<Action>();
		this.sequence = new LinkedList<Action>();
		this.name = name;
		this.drawInfo = new ActionDrawInformation(this);
		this.identifier = identifier;
	}
	
	/**
	 * Get list of properties.
	 * Value is an array of strings because a property may have multiple values
	 * @return list of action properties
	 */
	public LinkedHashMap<String, List<String>> getProperties() {
		return properties;
	}

	/**
	 * Overwrite current list of properties
	 * Value is an array of strings because a property may have multiple values
	 * @param properties new properties list
	 */
	public void setProperties(LinkedHashMap<String, List<String>> properties) {
		this.properties = properties;
		drawInfo.notifyModified();
	}
	
	/**
	 * Get property with specified key
	 * @param key Key of property
	 * @return Value of property or null if not found
	 */
	public List<String> getProperty(String key)
	{
		return properties.get(key);
	}
	
	/**
	 * Set or overwrite property with specified key
	 * @param key key to set
	 * @param value value to set
	 */
	public void setProperty(String key, String value)
	{
	    List<String> list = properties.get(key);
	    if(list == null){
	        list = new ArrayList<String>();
	        properties.put(key, list);
	    }
	    list.add(value);

		drawInfo.notifyModified();
	}

	/**
	 * Get iterator for parent actions.
	 * To add or remove a parent action use dedicated methods
	 * @return Iterator for parent actions
	 */
	public Iterator<Action> getParentActionsIterator() {
		return parentActions.iterator();
	}

	/**
	 * Add a parent action to this action.
	 * This action will be added automatically to the parentAction's child list
	 * @param parentAction Action to add
	 */
	public void addParentAction(Action parentAction) {
		this.parentActions.add(parentAction);
		if (parentAction.childActions.contains(this) == false)
		{
			parentAction.childActions.add(this);
			parentAction.drawInfo.notifyModified();
		}
		drawInfo.notifyModified();
	}
	
	/**
	 * Add a list of parent actions to this action.
	 * This action will be added automatically to the each parentAction's child list
	 * @param parentAction Actions to be added
	 */
	public void addParentActions(LinkedList<Action> parentActions) {
		this.parentActions.addAll(parentActions);
		for (Action parent : parentActions)
		{
			if (parent.childActions.contains(this) == false)
			{
				parent.childActions.add(this);
				parent.drawInfo.notifyModified();
			}
		}
		drawInfo.notifyModified();
	}
	
	/**
	 * Remove specified action from parents list.
	 * This action will be automatically removed from parentAction's child list
	 * @param parentAction action to remove
	 */
	public void removeParentAction(Action parentAction)
	{
		this.parentActions.remove(parentAction);
		parentAction.childActions.remove(this);
		drawInfo.notifyModified();
		parentAction.drawInfo.notifyModified();
	}

	/**
	 * Get iterator for child actions.
	 * To add or remove a child action use dedicated methods
	 * @return Iterator for child actions
	 */
	public Iterator<Action> getChildActionsIterator() {
		return childActions.iterator();
	}
	
	/**
	 * Add a child action to this action.
	 * This action will be added automatically to the childAction's parent list
	 * @param childAction Action to add
	 */
	public void addChildAction(Action childAction) {
		this.childActions.add(childAction);
		if (childAction.parentActions.contains(this) == false)
		{
			childAction.parentActions.add(this);
			childAction.drawInfo.notifyModified();
		}
		drawInfo.notifyModified();
	}

	/**
	 * Add a list of child actions to this action.
	 * This action will be added automatically to each childAction's parent list
	 * @param childActions Action list to add
	 */
	public void addChildActions(LinkedList<Action> childActions) {
		this.childActions.addAll(childActions);
		for (Action child : childActions)
		{
			if (child.parentActions.contains(this) == false)
			{
				child.parentActions.add(this);
				child.drawInfo.notifyModified();
			}
		}
		drawInfo.notifyModified();
	}
	
	/**
	 * Remove specified action from child list.
	 * This action will be automatically removed from childAction's parent list
	 * @param childAction action to remove
	 */
	public void removeChildAction(Action childAction)
	{
		this.childActions.remove(childAction);
		childAction.parentActions.remove(this);
		childAction.drawInfo.notifyModified();
		drawInfo.notifyModified();
	}

	/**
	 * Get iterator for action subsequences.
	 * To add or remove a sequence use dedicated methods.
	 * @return Iterator for sequences list
	 */
	public Iterator<Action> getSequenceIterator() {
		return sequence.iterator();
	}

	/**
	 * Returns the number of subsequences for this action
	 * @return length of sequence iterator
	 */
	public int getSequenceCount()
	{
		return sequence.size();
	}
	
	/**
	 * Add an action to the sequence list as a subsequence for this action.
	 * @param sequence action to add
	 */
	public void addSequence(Action sequence) {
		this.sequence.add(sequence);
		sequence.parentOfSequence = this;
		drawInfo.notifyModified();
	}
	
	/**
	 * Add multiple actions to the sequence list as subsequences for this action.
	 * @param sequences List of actions to add to sequence list
	 */
	public void addSequence(LinkedList<Action> sequences) {
		this.sequence.addAll(sequences);
		for (Action s : sequences)
		{
			s.parentOfSequence = this;
		}
		drawInfo.notifyModified();
	}
	
	/**
	 * Remove action from subsequence list
	 * @param sequence sequence to remove
	 */
	public void removeSequence(Action sequence)
	{
		this.sequence.remove(sequence);
		drawInfo.notifyModified();
	}
	
	/**
	 * Return name of action
	 * @return name
	 */
	public String getName()
	{
		return name;
	}
	
	/**
	 * Return identifier of the action	
	 * @return identifier
	 */
	public String getIdentifier() {
		return identifier;
	}

	/**
	 * Get information for drawing the action
	 * @return object containing draw information
	 */
	public ActionDrawInformation getDrawInfo()
	{
		return drawInfo;
	}
	
	/**
	 * Return number of parent actions
	 * @return number of parent actions
	 */
	public int getParentsCount()
	{
		return parentActions.size();
	}
	
	/**
	 * Return number of child actions
	 * @return number of child actions
	 */
	public int getChildrenCount()
	{
		return childActions.size();
	}
	
	/**
	 * Returns true if the sequence is expanded.
	 * @return true if expanded
	 */
	public boolean isExpanded()
	{
		if(parentOfSequence!=null)
			return (parentOfSequence.expandedSequence == this);
		else 
			return false;
	}
	
	/**
	 * Get the currently expanded subsequence
	 * @return Action which is currently expanded
	 */
	public Action getExpandedSequence()
	{
		return expandedSequence;
	}
	
	/**
	 * Expand the given action in this actions subsequence
	 * @param sequence Subcation to expand
	 */
	public void setExpandedSequence(Action sequence)
	{
		if (this.sequence.contains(sequence))
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
