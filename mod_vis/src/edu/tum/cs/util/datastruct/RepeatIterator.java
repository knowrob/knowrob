/*
 * Created on May 21, 2010
 *
 * TODO To change the template for this generated file go to
 * Window - Preferences - Java - Code Style - Code Templates
 */
package edu.tum.cs.util.datastruct;

import java.util.Iterator;
import java.util.NoSuchElementException;

public class RepeatIterator<T> implements Iterable<T> {

	protected T item;
	protected int repetitions;
	
	public RepeatIterator(T item, int repetitions) {
		this.item = item;
		this.repetitions = repetitions;
	}
	
	public Iterator<T> iterator() {		
		return new LocalIterator();
	}
	
	protected class LocalIterator implements Iterator<T> {

		protected int i;
		
		public LocalIterator() {
			this.i = 0;
		}
		
		public boolean hasNext() {			
			return i < repetitions;
		}

		public T next() {
			if(!hasNext())
				throw new NoSuchElementException();
			++i;
			return item;		
		}

		public void remove() {
			throw new RuntimeException("remove not supported");
		}
		
	}
}
