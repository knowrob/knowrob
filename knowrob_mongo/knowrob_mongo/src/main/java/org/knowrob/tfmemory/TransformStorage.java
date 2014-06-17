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

import java.util.Date;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import tfjava.StampedTransform;

import com.mongodb.BasicDBObject;

/**
 * Transformation stamped with time, frame ID and child frame ID, as it is stored in the buffer.
 *
 * A transformation object of this class is internally represented by a translation vector and
 * rotation quaternion. Since tf messages are also represented this way, no expensive conversion
 * takes place if a tf is stored in the buffer. Once a transform of this type is needed for actual
 * transformation, it is converted to a {@link StampedTransform} object that is represented by a 4x4 matrix.
 *
 * @author Sjoerd van den Dries
 * @version March 3, 2011
 */
public class TransformStorage {

    /** Reference to the parent frame (source frame) */
    protected Frame parentFrame;

    /** Reference to the child frame (source frame) */
    protected Frame childFrame;

    /** Time stamp in nanoseconds */
    protected long timeStamp;

    /** Translation vector */
    protected Vector3d translation;

    /** Rotation quaternion */
    protected Quat4d rotation;

    /** Time stamp of last access to this datastructure in nanoseconds */
    protected long lastAccessed;


	/**
	 * Normal constructor
	 *
	 * @param translation
	 * @param rotation
	 * @param timeStamp
	 * @param parentFrame
	 * @param childFrame
	 */
    public TransformStorage(Vector3d translation, Quat4d rotation, long timeStamp, Frame parentFrame, Frame childFrame) {
        this.childFrame = childFrame;
        this.parentFrame = parentFrame;
        this.timeStamp = timeStamp;
        this.translation = translation;
        this.rotation = rotation;

        this.lastAccessed = new Date().getTime();
    }

    /**
     * Simple constructor. To be used with "readFromDBObject()
     */
    public TransformStorage() {
    	this(new Vector3d(), new Quat4d(), 0l, new Frame("", 0), new Frame("", 0));
    }


    /**
     * Returns a new TransformStorage object that is an interpolation between or (forward or backward)
     * extrapolation from t1 and t2.
     */
    public static TransformStorage interpolate(TransformStorage t1, TransformStorage t2, long time) {
        long time1 = t1.getTimeStamp();
        long time2 = t2.getTimeStamp();

        // Check for zero distance case
        if (time1 == time2) {
          return t1;
        }

        //Calculate the ratio
        double ratio = (double)(time - time1) / (time2 - time1);

        // interpolate translation
        Vector3d transRet = new Vector3d();
        transRet.interpolate(t1.getTranslation(), t2.getTranslation(), ratio);

        // interpolate rotation
        Quat4d rotRet = new Quat4d();
        rotRet.interpolate(t1.getRotation(), t2.getRotation(), ratio);

        // original tf implementation (see cache.cpp) does not 'interpolate' timestamp. I do.

        return new TransformStorage(transRet, rotRet, time, t1.parentFrame, t1.childFrame);
    }

    /**
     * Returns a reference to the child (i.e., target) frame
     */
	public Frame getChildFrame() {
    	this.lastAccessed = new Date().getTime();
        return childFrame;
    }

    /**
     * Returns a reference to the parent (i.e., source) frame
     */
	public Frame getParentFrame() {
    	this.lastAccessed = new Date().getTime();
        return parentFrame;
    }

    /**
     * Returns the time stamp of this transform, in nanoseconds.
     */
	public long getTimeStamp() {
    	this.lastAccessed = new Date().getTime();
        return timeStamp;
    }

    /**
     * Returns the translation vector
     */
	public Vector3d getTranslation() {
    	this.lastAccessed = new Date().getTime();
        return translation;
    }

    /**
     * Returns the rotation quaternion
     */
	public Quat4d getRotation() {
    	this.lastAccessed = new Date().getTime();
        return rotation;
    }

    /**
     * Returns the time stamp of the last access to this transform, in milliseconds.
     */
    public long getLastAccessed() {
        return lastAccessed;
    }

	/**
	 * Read from a MongoDB result object
	 *
	 * @param row BasicDBObject, e.g. result of a MongoDB query
	 * @return Instance of TransformStorage with values from row
	 */
	public TransformStorage readFromDBObject(BasicDBObject transform, Frame childFrame, Frame parentFrame) {

		this.childFrame  = childFrame;
		this.parentFrame = parentFrame;

		BasicDBObject pos = (BasicDBObject) ((BasicDBObject) transform.get("transform")).get("translation");
		this.translation.x = pos.getDouble("x");
		this.translation.y = pos.getDouble("y");
		this.translation.z = pos.getDouble("z");

		BasicDBObject rot = (BasicDBObject) ((BasicDBObject) transform.get("transform")).get("rotation");
		this.rotation.x = rot.getDouble("x");
		this.rotation.y = rot.getDouble("y");
		this.rotation.z = rot.getDouble("z");
		this.rotation.w = rot.getDouble("w");

		this.timeStamp = (long) (((Date) ((BasicDBObject) transform.get("header")).get("stamp")).getTime() * 1E6);

		return this;
	}

}
