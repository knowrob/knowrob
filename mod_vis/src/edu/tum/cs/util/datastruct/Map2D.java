package edu.tum.cs.util.datastruct;

import java.util.HashMap;

/**
 * a mapping with two-dimensional keys (context/key) 
 * @author jain
 */
public class Map2D<TContext, TKey, TValue> {
	/**
	 * the actual two-dimensional mapping
	 */
	protected HashMap<TContext, HashMap<TKey, TValue>> map;
	/**
	 * the last-used submap
	 */
	protected HashMap<TKey, TValue> submap;
	/**
	 * the key in the current submap that was last retrieved
	 */
	protected TKey key;
	
	public Map2D() {
		map = new HashMap<TContext, HashMap<TKey, TValue>>();			
	}
	
	/**
	 * retrieves the value for a particular context/key combination
	 * @param context
	 * @param key
	 * @return
	 */
	public TValue get(TContext context, TKey key) {
		this.key = key;
		submap = map.get(context);
		if(submap == null) {
			submap = new HashMap<TKey, TValue>();
			map.put(context, submap);
			return null;
		}		
		return submap.get(key);
	}
	
	/**
	 * retrieves the key-value mapping for a particular context 
	 * @param context
	 * @return
	 */
	public HashMap<TKey, TValue> getSubmap(TContext context) {
		return map.get(context);
	}
	
	/**
	 * stores a new value for the last-retrieved context/key combination
	 * @param value
	 */
	public void put(TValue value) {
		submap.put(key, value);
	}
	
	/**
	 * stores the given value under the given context/key combination
	 * @param context
	 * @param key
	 * @param value
	 */
	public void put(TContext context, TKey key, TValue value) {
		submap = map.get(context);
		if(submap == null) {
			submap = new HashMap<TKey, TValue>();
			map.put(context, submap);			
		}
		submap.put(this.key = key, value);
	}
}