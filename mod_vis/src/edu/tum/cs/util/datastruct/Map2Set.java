package edu.tum.cs.util.datastruct;

import java.util.HashMap;
import java.util.Set;
import java.util.Vector;
import java.util.Map.Entry;

/**
 * represents a mapping from keys to lists of values (specifically java.util.Vectors of values)
 * @author jain
 */
public class Map2Set<TKey, TValue> {
	/**
	 * the actual mapping
	 */
	protected HashMap<TKey, Set<TValue>> map;
	
	public Map2Set() {
		map = new HashMap<TKey, Set<TValue>>();			
	}
	
	/**
	 * gets the list of values stored for the given key
	 * @param key
	 * @return a vector of values or null of no values were saved yet for the given key
	 */
	public Set<TValue> get(TKey key) {
		return map.get(key);
	}
	
	/**
	 * adds to the list managed for key the given value, creating the list if it doesn't exist yet
	 * @param key
	 * @param value
	 */
	public void add(TKey key, TValue value) {
		Set<TValue> v = map.get(key);
		if(v == null) {
			v = new java.util.HashSet<TValue>();
			map.put(key, v);			
		}
		v.add(value);
	}
	
	public Set<Entry<TKey, Set<TValue>>> entrySet() {
		return map.entrySet();
	}
}