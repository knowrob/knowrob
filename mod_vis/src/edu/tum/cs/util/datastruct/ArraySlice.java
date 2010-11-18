/*
 * Created on Nov 5, 2009
 *
 * TODO To change the template for this generated file go to
 * Window - Preferences - Java - Code Style - Code Templates
 */
package edu.tum.cs.util.datastruct;

/**
 * generic array slicing
 * @author jain
 */
public class ArraySlice<T> implements Iterable<T> {
	T[] a;
	int iFirst, iLast;
	
	/**
	 * @param a the array to slice
	 * @param first the index of the first element to include; if negative, then -n stands for the index of n-th element from the end of the list
	 * @param end the index of the last element not to include; if negative, then -n means that the last n elements are to be omitted 
	 */
	public ArraySlice(T[] a, int first, int end) {
		this.a = a;
		iFirst = first;
		if(iFirst < 0)
			iFirst += a.length;
		if(end < 0)
			end += a.length;
		else 
			if(end > a.length)
				end = a.length;
		iLast = end-1;
	}
	
	public ArraySlice(T[] a, int first) {
		this(a, first, a.length);
	}

	public java.util.Iterator<T> iterator() {		
		return new Iterator();
	}
	
	protected class Iterator implements java.util.Iterator<T> {
		protected int iNext;
		
		public Iterator() {
			iNext = iFirst;
		}
		
		public boolean hasNext() {			
			return iNext <= iLast;
		}
	
		public T next() {
			return a[iNext++];			
		}
	
		public void remove() {
			throw new RuntimeException("remove is not supported by this iterator");		
		}
	}
}