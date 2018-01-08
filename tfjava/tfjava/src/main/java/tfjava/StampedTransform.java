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

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Point3d;
import javax.vecmath.Matrix4d;

/** 
 * Transformation stamped with time, frame ID and child frame ID. 
 *  
 * A transformation object of this class is internally represented by a 4x4 matrix.  
 *  
 * @author Sjoerd van den Dries
 * @version Feb 28, 2011
 */
public class StampedTransform {       
        
    /** name of the parent frame (source frame) */    
    public String frameID;
    /** name of the child frame (target frame) */
    public String childFrameID;
    /** Time stamp */
    public Time timeStamp;
    /** Transformation matrix representing the transform */ 
    protected Matrix4d transform;
        
    /**
     * Class constructor.
     */    
    public StampedTransform(Vector3d translation, Quat4d rotation, Time timeStamp, String frameID, String childFrameID) {
        this.childFrameID = childFrameID; 
        this.frameID = frameID;
        this.timeStamp = timeStamp;        
        this.transform = new Matrix4d(rotation, translation, 1);
    }
    
    /**
     * Class constructor.
     */    
    public StampedTransform(Matrix4d transform, Time timeStamp, String frameID, String childFrameID) {
        this.childFrameID = childFrameID; 
        this.frameID = frameID;
        this.timeStamp = timeStamp;
        this.transform = transform;
    }
    
    /**
     * Returns an identity transformation object.
     */    
    public static StampedTransform getIdentity() {
        Matrix4d identity = new Matrix4d();
        identity.setIdentity();        
        return new StampedTransform(identity, null, null, null);
    }
    
    /**
     * Inverts this transformation in place.
     */    
    public StampedTransform invert() {
        transform.invert();
        String mem = this.childFrameID;
        this.childFrameID = frameID;
        this.frameID = mem;
        return this;
    }
    
    /**
     * Sets the value of this transform to the result of multiplying itself with transform t1.
     */
    public void mul(StampedTransform t1) {        
        this.transform.mul(t1.transform);
        this.childFrameID = t1.childFrameID;
    }

    /**
     * Sets the value of this transform to the result of multiplying t1 with t2.
     */    
    public void mul(StampedTransform t1, StampedTransform t2) {
        this.transform.mul(t1.transform, t2.transform);
        this.frameID = t1.frameID;
        this.childFrameID = t2.childFrameID;
    }
    
    /**
     * Transforms the pose parameter with this transform and places the result into poseOut.
     */    
    public void transformPose(Matrix4d pose, Matrix4d poseOut) {
        poseOut.mul(getMatrix4(), pose);
    }
    
    /**
     * Transforms the point parameter with this transform and places the result into pointOut.
     */    
    public void transformPoint(Point3d point, Point3d pointOut) {
        getMatrix4().transform(point, pointOut);
    }
    
    /**
     * Transforms the vector parameter with this transform and places the result into vectorOut.
     */    
    public void transformVector(Vector3d vector, Vector3d vectorOut) {
        getMatrix4().transform(vector, vectorOut);
    }
    
    /**
     * Transforms the stamped pose stampedIn with this transform and places the result into stampedOut.
     */  
    public void transformPose(Stamped<Matrix4d> stampedIn, Stamped<Matrix4d> stampedOut) {
        if (stampedIn.frameID != frameID) {
            // TODO: throw error
            return;
        }
        transformPose(stampedIn.getData(), stampedOut.getData());
        stampedOut.frameID = childFrameID;
        stampedOut.timeStamp = new Time(timeStamp);
    }
    
    /**
     * Transforms the stamped point stampedIn with this transform and places the result into stampedOut.
     */  
    public void transformPoint(Stamped<Point3d> stampedIn, Stamped<Point3d> stampedOut) {
        if (stampedIn.frameID != frameID) {
            // TODO: throw error
            return;
        }
        transformPoint(stampedIn.getData(), stampedOut.getData());
        stampedOut.frameID = childFrameID;
        stampedOut.timeStamp = new Time(timeStamp);
    }  
    
    /**
     * Transforms the stamped vector stampedIn with this transform and places the result into stampedOut.
     */
    public void transformVector(Stamped<Vector3d> stampedIn, Stamped<Vector3d> stampedOut) {
        if (stampedIn.frameID != frameID) {
            // TODO: throw error
            return;
        }
        transformVector(stampedIn.getData(), stampedOut.getData());
        stampedOut.frameID = childFrameID;
        stampedOut.timeStamp = new Time(timeStamp);
    }          
 
    /**
     * Calculates and returns the translational component of this transform.
     */
    public Vector3d getTranslation() {
        Vector3d out = new Vector3d();
        transform.get(out);
        return out;
    }
    
    /**
     * Calculates and returns the translational component of this transform in parameter out.
     */
    public void getTranslation(Vector3d out) {
        transform.get(out);
    }    
    /**
     * Calculates and returns the rotational component of this transform.
     */
    public Quat4d getRotation() {
        Quat4d out = new Quat4d();
        transform.get(out);
        return out;
    }
    
    /**
     * Calculates and returns the rotational component of this transform in parameter out.
     */
    public void getRotation(Quat4d out) {
        transform.get(out);
    }
    
    /**
     * Returns the 4x4 matrix representing this transform.
     */
    public Matrix4d getMatrix4() {
        return transform;
    }      
    
    /**
     * Returns a string that contains the values of this transform.
     */    
    public String toString() {
        return "[" + frameID + " -> " + childFrameID + ", " + timeStamp + ", " + getMatrix4() + "]";
    } 
    
}
