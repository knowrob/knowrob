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

import org.ros.message.Time;

/**
 * Wrapper class adding a frame ID and time stamp to data of type T.
 * 
 * @author Sjoerd van den Dries
 * @version Feb 28, 201
 */
public class Stamped<T> {
    
    /** Data to be stamped. */
    protected T data;        
    /** Frame ID of the data. */
    public String frameID;
    /** Time stamp of the data. */
    public Time timeStamp;
    
    /**
     * Constructs and initializes an empty stamp (all nulls).
     */    
    public Stamped() {
        this.data = null;
        this.frameID = null;
        this.timeStamp = null;
    }

    /**
     * Constructs a wrapper for data stamped with frameID and timeStamp.
     */    
    public Stamped(T data, String frameID, Time timeStamp) {
        this.data = data;
        this.frameID = frameID;
        this.timeStamp = timeStamp;
    }
    
    /**
     * Constructs a wrapper for data stamped with frameID and timeStamp.
     */    
    public Stamped(T data, String frameID, double posix_ts) {
    	
		Time t = new Time();
		t.secs = (int)posix_ts;
		t.nsecs = (int) (1E9 * (posix_ts - ((int) posix_ts)));
		
        this.data = data;
        this.frameID = frameID;
        this.timeStamp = t;
    }
    
    /**
     * Returns data.
     */    
    public T getData() {
        return data;
    }
    
    /**
     * Sets data.
     */
    public void setData(T data) {
        this.data = data;
    }
    
}
