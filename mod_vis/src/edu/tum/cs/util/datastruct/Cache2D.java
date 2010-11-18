package edu.tum.cs.util.datastruct;

public class Cache2D<TContext, TKey, TValue> extends Map2D<TContext, TKey, TValue> {
	public int numCacheHit = 0;
	public int numCacheMiss = 0;
	
	public TValue get(TContext context, TKey key) {
		TValue value = super.get(context, key);
		if(value == null)
			numCacheMiss++;
		else 
			numCacheHit++;
		return value;
	}
	
	public float getHitRatio() {
		return (float)numCacheHit/getNumAccesses();
	}
	
	public int getNumAccesses() {
		return numCacheHit+numCacheMiss;
	}
}