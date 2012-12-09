/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.util;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * This is an implementation of a HashMap which can contain multiple values for a single key. So if
 * there is inserted an element with the same key, the elements value is added to the current
 * content in a list.
 * 
 * 
 * @author Stefan Profanter
 * @param <K>
 *            The key type
 * @param <V>
 *            The value type
 * 
 */
public class MultiValueHashMap<K, V> implements MultiValueMap<K, V> {
	/**
	 * The internal map containing the list
	 */
	private final Map<K, Set<V>>	map	= new HashMap<K, Set<V>>();

	@Override
	public void clear() {
		map.clear();
	}

	@Override
	public boolean containsKey(K key) {
		return map.containsKey(key);
	}

	@Override
	public boolean containsValue(V value) {
		for (K key : map.keySet()) {
			Set<V> values = map.get(key);
			if (values.contains(value)) {
				return true;
			}
		}
		return false;
	}

	@Override
	public boolean equals(Object obj) {
		if (obj == null) {
			return false;
		}
		if (!(obj instanceof MultiValueMap)) {
			return false;
		}
		@SuppressWarnings("unchecked")
		final MultiValueMap<K, V> other = (MultiValueMap<K, V>) obj;
		if (this.keySet() != other.keySet()
				&& (this.keySet() == null || !this.keySet().equals(other.keySet()))) {
			return false;
		}
		if (this.values() != other.values()
				&& (this.values() == null || !this.values().equals(other.values()))) {
			return false;
		}
		return true;
	}

	@Override
	public Collection<V> get(K key) {
		return map.get(key);
	}

	@Override
	public int hashCode() {
		int hash = 5;
		hash = 71 * hash + (this.map != null ? this.map.hashCode() : 0);
		return hash;
	}

	@Override
	public boolean isEmpty() {
		if (size() == 0)
			return true;
		return false;
	}

	@Override
	public Set<K> keySet() {
		return map.keySet();
	}

	@Override
	public Collection<V> put(K key, V value) {
		Set<V> l = map.get(key);
		if (l == null) {
			l = new LinkedHashSet<V>();
		}
		l.add(value);
		return map.put(key, l);
	}

	@Override
	public Collection<V> remove(K key) {
		return map.remove(key);
	}

	@Override
	public int size() {
		return map.size();
	}

	@Override
	public String toString() {
		return map.toString();
	}

	@Override
	public Collection<V> values() {
		List<V> values = new ArrayList<V>();
		for (K key : map.keySet()) {
			Set<V> v = map.get(key);
			values.addAll(v);
		}
		return values;
	}
}

/**
 * Interface for a MultiValue map
 * 
 * @author Stefan Profanter
 * 
 * @param <K>
 * @param <V>
 */
interface MultiValueMap<K, V> {

	/**
	 * Empty map
	 */
	void clear();

	/**
	 * Check if map contains key
	 * 
	 * @param key
	 *            the key to check
	 * @return true if map contains key
	 */
	boolean containsKey(K key);

	/**
	 * Check if map contains value
	 * 
	 * @param value
	 *            the value to check
	 * @return true if map contains value
	 */
	boolean containsValue(V value);

	/**
	 * Get all values for given key contained in the map
	 * 
	 * @param key
	 *            the identifier
	 * @return all values found or null
	 */
	Collection<V> get(K key);

	/**
	 * Check if map is empty
	 * 
	 * @return true if map is empty
	 */
	boolean isEmpty();

	/**
	 * Get all keys
	 * 
	 * @return set containing all keys
	 */
	Set<K> keySet();

	/**
	 * Insert a new value with given key into the map. If the key already exists, the old value is
	 * NOT overwritten. The new value is added to the list instead.
	 * 
	 * @param key
	 *            key of the object
	 * @param value
	 *            the value to add
	 * @return the previous values associated with key.
	 * 
	 */
	Collection<V> put(K key, V value);

	/**
	 * Remove all values of given key.
	 * 
	 * @param key
	 *            key to identify values.
	 * @return the values for this key
	 */
	Collection<V> remove(K key);

	/**
	 * Get the size of the map which is the number of keys.
	 * 
	 * @return number of keys
	 */
	int size();

	/**
	 * Get all values of all keys
	 * 
	 * @return list of all values.
	 */
	Collection<V> values();

}
