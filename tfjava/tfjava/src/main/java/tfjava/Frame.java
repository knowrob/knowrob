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

import java.util.HashMap;
import java.util.Set;

/**
 * Transformation frame, containing references to all parent frames and corresponding time caches.
 * 
 * @author Sjoerd van den Dries
 * @version Feb 28, 2011
 */
public class Frame {
    
    /** ID of this frame. */
    private String frameID;
    /** Map containing parent frames of this frame with corresponding time caches.*/
    private HashMap<Frame, TimeCache> parentMap;
    /** Maximum storage time of time caches in nanoseconds.*/    
    private long maxStorageTime;
    
    /**
     * Class constructor.
     */
    public Frame(String frameID, long maxStorageTime) {
        this.frameID = frameID;
        this.maxStorageTime = maxStorageTime;
        parentMap = new HashMap<Frame, TimeCache>();
    }
    
    /**
     * Inserts transform newData into the appropriate time cache (based on frame of newData). 
     */
    public boolean insertData(TransformStorage newData) {
        Frame childFrame = newData.getChildFrame();
        
        if (childFrame != this) {
            // new data is not inserted in the right frame. TODO: throw error
            return false;
        }
        
        Frame parentFrame = newData.getParentFrame();
        TimeCache cache = parentMap.get(parentFrame);
        
        if (cache == null) {
            cache = new TimeCache(maxStorageTime, parentFrame, childFrame);
            parentMap.put(parentFrame, cache);
        }
        
        return cache.insertData(newData);
    }
    
    /**
     * Retrieves the direct transformation from sourceFrame to this frame, at time point time.
     * 
     * Parameter sourceFrame must be a direct parent of this frame. The transformation is looked up
     * in the appropriate time cache, if available. If necessary, inter- or extrapolation is used
     * to return a transformation of the given point in time.
     */
    public TransformStorage getData(long time, Frame sourceFrame) {
        TimeCache cache = parentMap.get(sourceFrame);
        
        if (cache == null) {
            // parent frame not found. TODO: throw error
            return null;
        }
        return cache.getData(time);
    }
    
    /**
     * Returns the ID of this frame.
     */
    public String getFrameID() {
        return frameID;
    }
    
    /**
     * Returns the time cache containing the transformations from sourceFrame to this frame.
     */
    public TimeCache getTimeCache(Frame sourceFrame) {
        return parentMap.get(sourceFrame);
    }
    
    /**
     * Returns a set containing references to all parent frames of this frame.
     */
    public Set<Frame> getParentFrames() {
        return parentMap.keySet();
    }
    
}
