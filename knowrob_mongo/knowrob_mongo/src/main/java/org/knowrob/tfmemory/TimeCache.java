/*
 * Copyright (c) 2011, Sjoerd van den Dries
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Technische Universiteit Eindhoven nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


package org.knowrob.tfmemory;

import java.util.Collection;
import java.util.Date;
import java.util.Iterator;
import java.util.TreeMap;


/**
 * Buffer in which transformations from one specific frame to another are stored, ordered in time.
 *
 * @author Sjoerd van den Dries, Moritz Tenorth
 * @version Feb 19, 2014
 */
public class TimeCache {

    /** Map containing the transformations, ordered in time */
    protected TreeMap<Long, TransformStorage> storage;

    /** Maximum storage time, in nanoseconds */
    protected long maxExtrapolationTime;

    /** Buffer for time of last garbage collection */
    protected long lastGarbageCollection = 0;

    /** garbage collection interval in seconds */
    protected long GARBAGE_COLLECTION_INTERVAL = 300;

    /**
     * Class Constructor.
     */
    public TimeCache(long maxStorageTime, Frame parentFrame, Frame childFrame) {
        this.maxExtrapolationTime = maxStorageTime;
        this.storage = new TreeMap<Long, TransformStorage>();
        this.lastGarbageCollection = new Date().getTime() * 1000000;
    }

    /**
     * Inserts transformation newData in the buffer, while maintaining the time ordering.
     */
    public boolean insertData(TransformStorage newData) {

// MT: disabled since this does not allow to add data that is older than data already in the buffer
//     which is something we want to do
//        if (!storage.isEmpty() && storage.firstKey() - maxStorageTime > newData.getTimeStamp()) {
//            return false;
//        }

        storage.put(newData.getTimeStamp(), newData);

        removeOldData();
        return true;
    }

    /**
     * Returns the transformation in this buffer at time point time (in nanoseconds);
     * Uses interpolation or (forward or backward) extrapolation.
     *
     * If only one transformation is available, this transformation is returned, regardless
     * of how far back or ahead the queried time point is compared to the time stamp of this transformation.
     *
     */
	public TransformStorage getData(long time) {

        if (storage.isEmpty()) {
            // TODO: throw error: "Cache for frame " + parentFrame.getFrameID() + " to " + childFrame.getFrameID() + " is empty";
            return null;
        } else if (storage.size() == 1) {
            // only one transform in cache, so return that one
            return (TransformStorage) storage.firstEntry().getValue();
        }

        TransformStorage low, high;

        if (time < storage.firstKey()) {

        	// do not extrapolate more than maxStorageTime
        	if(storage.firstKey() - time > maxExtrapolationTime)
        		return null;

            // extrapolate back: low = oldest transform
            //                   high = oldest but one
            low = (TransformStorage) storage.firstEntry().getValue();                     // O(1) (I guess?)
            high = (TransformStorage) storage.higherEntry(storage.firstKey()).getValue(); // O(log n) --> can be done in O(1), but how?


        } else if (time > storage.lastKey()) {

        	// do not extrapolate more than maxStorageTime
        	if(time - storage.lastKey() > maxExtrapolationTime)
        		return null;

            // extrapolate forward: low = newest but one
            //                      high = newest transform
            low = (TransformStorage) storage.lowerEntry(storage.lastKey()).getValue();    // O(log n) --> can be done in O(1), but how?
            high = (TransformStorage) storage.lastEntry().getValue();                     // O(1) (I guess?)

        } else {

            // TODO: this causes the gaps in the trajectories -- do we need this?
        	// only interpolate if both the smaller and larger time point are closer than maxStorageTime
        	if((storage.ceilingEntry(time).getKey() - time > maxExtrapolationTime) &&
    		   (time - storage.floorEntry(time).getKey() > maxExtrapolationTime))
        		return null;
        	
            // interpolate: low = newest transform older than time,
            //              high = oldest transform newer than time
            high = (TransformStorage) storage.ceilingEntry(time).getValue();
            low = (TransformStorage) storage.floorEntry(time).getValue();
        }

        return TransformStorage.interpolate(low, high, time);

    }

    /**
     * Returns the absolute time difference to the nearest transform from the given
     * time point, in nanoseconds.
     */
    public long timeToNearestTransform(long time) {
        Long floor = storage.floorKey(time);
        Long ceiling = storage.ceilingKey(time);

        if (floor == null) return (ceiling - time);
        if (ceiling == null) return (time - floor);
        return Math.min(ceiling - time, time - floor);
    }

    /**
     * Removes all transforms that have not been accessed in the last maxStorageTime seconds
     */
	protected void removeOldData() {

    	long now = new Date().getTime() * 1000000;
        if (!storage.isEmpty() && (lastGarbageCollection < now - (GARBAGE_COLLECTION_INTERVAL * 1000000000))) {

        	long timeLowerbound = now - maxExtrapolationTime;

        	Collection<TransformStorage> values = storage.values();
        	synchronized(storage) {
        		Iterator<TransformStorage> i = values.iterator(); // Must be in synchronized block
        		while (i.hasNext()) {
        			TransformStorage t = i.next();

        			if((t instanceof TransformStorage) && (((TransformStorage) t).getLastAccessed() < timeLowerbound)) {
        				i.remove();
        			}
        		}

        	}

            lastGarbageCollection = now;
        }
    }

    /**
     * Check whether timepoint 'time' is between the oldest and newest entries in tbe buffer.
     *
     * @param time Time point to be checked
     * @return true if 'time' is between the oldest and newest buffer entry
     */
    public boolean timeInBufferRange(long time) {
    	if (!storage.isEmpty()) {
            if(time >  storage.firstKey()  && time < storage.lastKey())
            	return true;
        }
    	return false;
    }
}
