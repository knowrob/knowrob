package edu.tum.cs.ias.knowrob.vis.actions;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Vector;


public class ActionTransitions implements Iterable<ActionTransition> {

	List<ActionTransition> transitions;
	Action startAction = null;

	public ActionTransitions() {
		this.transitions = Collections.synchronizedList(new ArrayList<ActionTransition>());
	}
	
	
	public synchronized boolean add(ActionTransition t) {
		
		if(!transitions.contains(t))
			transitions.add(t);
		
		return true;
	}
	
	public synchronized boolean addAll(Collection<ActionTransition> ts) {
		return transitions.addAll(ts);
	}
	
	public synchronized boolean addAll(ActionTransitions ts) {
		return transitions.addAll(ts.transitions);
	}
	
	
	public synchronized void remove(ActionTransition t) {
		transitions.remove(t);
	}

	
	public synchronized void removeAll(Collection<ActionTransition> ts) {
		transitions.removeAll(ts);
	}
	
	
	public synchronized Set<ActionTransition> getTransitionsFrom(Action a) {
		
		Set<ActionTransition> res = new HashSet<ActionTransition>();
		
		for(ActionTransition t: transitions) {
			
			if(t.from.equals(a))
				res.add(t);
		}
		return res;
	}
	
	
	public synchronized Set<ActionTransition> getTransitionsTo(Action a) {
		
		Set<ActionTransition> res = new HashSet<ActionTransition>();
		
		for(ActionTransition t: transitions) {
			
			if(t.to.equals(a))
				res.add(t);
		}
		return res;
	}
	
	
	public synchronized int getTreeLevel(Action a) {

		int level = -1;
		if(startAction != null) {
			level = getTreeLevelRecursive(a, 0, new HashSet<Action>());
		}
		return level;
	}
	
	
	private synchronized int getTreeLevelRecursive(Action a, int level, Set<Action> visited) {
		
		if(a.equals(startAction)) {
			return level;
			
		} else {
			
			ArrayList<Integer> levels = new ArrayList<Integer>();
			for(ActionTransition trans : getTransitionsTo(a)) {
				
				if(!visited.contains(trans.from)) {
					visited.add(trans.from);
					levels.add(getTreeLevelRecursive(trans.from, ++level, visited));
				}
			}
			
			if(levels.size()>0)
				return Collections.min(levels);
			
			// if no parent can be found in this branch, return large value
			else return Integer.MAX_VALUE;
		}
	}
	
	
	public synchronized Map<Integer, Vector<Action>> getTreeLevels() {
		
		HashMap<Integer, Vector<Action>> levels = new HashMap<Integer, Vector<Action>>();
		for(ActionTransition t : transitions) {
			
			for(Action a : new Action[]{t.from, t.to}) {
				
				int l = getTreeLevel(a);
				if(!levels.containsKey(l)) {
					levels.put(l, new Vector<Action>());
				}
				if(!levels.get(l).contains(a))
					levels.get(l).add(a);
			}
		}
		return levels;
	}
	
	/**
	 * compute the maximum width of the actions to be drawn in order to compute the horizontally centered alignment
	 * @return
	 */
	public synchronized Map<Integer, Float> getLevelWidths() {
		
		Map<Integer, Float> levelWidths = new HashMap<Integer, Float>();
		Map<Integer, Vector<Action>> levels = getTreeLevels();
		
		for(int l : levels.keySet()) {
			
			Vector<Action> acts = levels.get(l);
			
			float width = 2* ActionDrawInformation.SEQUENCE_BOX_PADDING;
			for(Action a : acts) {
				width += a.getDrawInfo().getSimpleBoxDimension().x + ActionDrawInformation.SEQUENCE_BOX_PADDING;
			}
			levelWidths.put(l, width);
		}
		return levelWidths;
	}
	

	
	public Action getStartAction() {
		return startAction;
	}


	public void setStartAction(Action startAction) {
		this.startAction = startAction;
	}

	@Override
	public synchronized Iterator<ActionTransition> iterator() {
		return transitions.iterator();
	}
}
