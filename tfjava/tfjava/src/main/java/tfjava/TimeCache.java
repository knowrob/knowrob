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


package tfjava;

import java.util.TreeMap;

/**
 * Buffer in which transformations from one specific frame to another are stored, ordered in time. 
 * 
 * @author Sjoerd van den Dries
 * @version Feb 28, 2011
 */
public class TimeCache {
    
    /** Map containing the transformations, ordered in time */
    protected TreeMap<Long, TransformStorage> storage;
    /** Maximum storage time, in nanoseconds */
    protected long maxStorageTime;
    /** Reference to the parent frame (source frame) */
    protected Frame parentFrame;
    /** Reference to the child frame (target frame) */
    protected Frame childFrame;
    
    /**
     * Class Constructor.
     */
    public TimeCache(long maxStorageTime, Frame parentFrame, Frame childFrame) {
        this.maxStorageTime = maxStorageTime;
        this.storage = new TreeMap<Long, TransformStorage>();
        this.parentFrame = parentFrame;
        this.childFrame = childFrame;
    }
    
    /**
     * Inserts transformation newData in the buffer, while maintaining the time ordering. 
     */
    public boolean insertData(TransformStorage newData) {  
//        // check if data is older than first frame in STORAGE - maxStorageTime
//        if (!storage.isEmpty() && storage.firstKey() - maxStorageTime > newData.getTimeStamp()) {
//            return false;                              
//        }
        
        storage.put(newData.getTimeStamp(), newData);       
        
        removeOldData(); // same as pruneList in time_cache.h        
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
            return storage.firstEntry().getValue();
        }        
        
        TransformStorage low, high;        
        
        if (time < storage.firstKey()) {
            // extrapolate back: low = oldest transform
            //                   high = oldest but one 
            low = storage.firstEntry().getValue();                     // O(1) (I guess?)
            high = storage.higherEntry(storage.firstKey()).getValue(); // O(log n) --> can be done in O(1), but how?
        } else if (time > storage.lastKey()) {
            // extrapolate forward: low = newest but one
            //                      high = newest transform        
            low = storage.lowerEntry(storage.lastKey()).getValue();    // O(log n) --> can be done in O(1), but how?
            high = storage.lastEntry().getValue();                     // O(1) (I guess?)
        } else {
            // interpolate: low = newest transform older than time,
            //              high = oldest transform newer than time
            low = storage.ceilingEntry(time).getValue();
            high = storage.floorEntry(time).getValue();
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
     * Removes all transforms that are more than maxStorageTime older than the newest transform.
     */
    protected void removeOldData() {
        if (!storage.isEmpty()) {
            long timeLowerbound = storage.lastKey() - maxStorageTime;
            while (!storage.isEmpty() && storage.firstKey() < timeLowerbound) {
                storage.pollFirstEntry();
            }       
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
