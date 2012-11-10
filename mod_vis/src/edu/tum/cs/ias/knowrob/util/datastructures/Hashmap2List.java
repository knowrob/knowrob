package edu.tum.cs.ias.knowrob.util.datastructures;

import java.util.HashMap;
import java.util.Vector;

public class Hashmap2List<KeyT, ValueT> {
	protected HashMap<KeyT, Vector<ValueT>> map = new HashMap<KeyT, Vector<ValueT>>();
	
	public void put(KeyT key, ValueT value) {
		Vector<ValueT> v;
		if(!map.containsKey(key)) { 
			v = new Vector<ValueT>();
			map.put(key, v);
		}
		else
			v = map.get(key);
		v.add(value);
	}
	
	public Vector<ValueT> get(KeyT key) {
		return map.get(key);
	}
}
