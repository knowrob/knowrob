/**
 * 
 */
package edu.tum.cs.util.datastruct;

import java.util.Iterator;

/**
 * CollectionFilter
 * 
 * represents an iterator that filters a Collection<CollectionType> for objects of type T
 * 
 * @author jain
 *
 * @param <T>  the type to filter for
 * @param <CollectionType>  the type of object found in the collection
 */
public class CollectionFilter<T, CollectionType> implements Iterable<T>, Iterator<T> {
	
	Iterator<CollectionType> iter;
	CollectionType item;
	Class<T> classObj;
	
	public CollectionFilter(Iterable<CollectionType> c, Class<T> classObj) {
		iter = c.iterator();			
		this.classObj = classObj;
		findNext();
	}
	
	public void findNext() {
		while(iter.hasNext()) {
			item = iter.next();
			if(item.getClass().equals(classObj)) {
				return;
			}		
		}
		item = null;
	}
	
	public Iterator<T> iterator() {
		return this;
	}

	public boolean hasNext() {			
		return item != null;
	}

	public T next() {
		@SuppressWarnings("unchecked")
		T ret = (T)item;
		findNext();
		return ret;
	}

	public void remove() {
		throw new RuntimeException("Remove is not supported by this iterator!");			
	}		
}