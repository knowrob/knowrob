/*
 * Created on Sep 10, 2010
 *
 * TODO To change the template for this generated file go to
 * Window - Preferences - Java - Code Style - Code Templates
 */
package edu.tum.cs.util.datastruct;

import java.util.Collection;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;
import java.util.Vector;

/**
 * a set that uses an iterator whose ordering is guaranteed to be equivalent to the order of insertion into the set.
 * The space requirements are doubled.   
 * @author jain
 */
public class OrderedSet<E> implements Set<E> {

	Vector<E> v = new Vector<E>();
	HashSet<E> s = new HashSet<E>();
	
	public boolean add(E e) {
		if(s.contains(e))			
			return false;
		v.add(e);
		s.add(e);
		return true;
	}

	public boolean addAll(Collection<? extends E> c) {
		boolean changed = false;
		for(E e : c)
			changed = changed || add(e);
		return changed;
	}

	public void clear() {
		v.clear();
		s.clear();
	}

	public boolean contains(Object o) {		
		return s.contains(o);
	}

	public boolean containsAll(Collection<?> c) {
		for(Object e : c)
			if(!contains(e))
				return false;
		return true;
	}

	public boolean isEmpty() {
		return v.isEmpty();
	}

	public Iterator<E> iterator() {		
		return v.iterator();
	}

	public boolean remove(Object o) {
		if(s.remove(o)) {
			v.remove(o);
			return true;
		}
		return false;
	}

	public boolean removeAll(Collection<?> c) {
		boolean removal = false;
		for(Object e : c)
			removal = removal || remove(e);
		return removal;
	}

	public boolean retainAll(Collection<?> c) {
		throw new RuntimeException("Operation not supported");
	}

	public int size() {
		return v.size();
	}

	public Object[] toArray() {
		return v.toArray();
	}

	public <T> T[] toArray(T[] a) {
		return v.toArray(a);
	}

}
