package tfjava;
/*
 * Copyright (c) 2011, Sjoerd van den Dries, Moritz Tenorth
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

import org.ros.message.Time;

import javax.vecmath.Point3d;
import javax.vecmath.Matrix4d;



/**
 * Utilities for using tfjava, especially for calling the 
 * methods from Prolog (see tf_prolog package)
 *
 * @author Sjoerd van den Dries, Moritz Tenorth
 * @version August 2, 2011
 */

public class Utils {
    
	/**
	 * Convert time stamp in seconds into ros.communication.Time
	 * 
	 * @param secs Time stamp in seconds
	 * @return ros.communication.Time object
	 */

	public static Time secondsToRosTime(double secs) {
	    return new Time(secs);
	}

	
	/**
	 * Convert a StampedTransform into an array of pose elements
	 * 
	 * @param tf   StampedTransform describing the transformation
	 * @return     Row-based array with the matrix elements
	 */
    public static double[] tfToPoseArray(StampedTransform tf) {
        Matrix4d m = tf.getMatrix4();
        double[] mArray = new double[] {m.m00, m.m01, m.m02, m.m03, m.m10, m.m11, m.m12, m.m13,
                                        m.m20, m.m21, m.m22, m.m23, m.m30, m.m31, m.m32, m.m33};
        return mArray;
    }
    

    /**
     * Convert a Stamped<Point3d> object to a point array [x,y,z] 
     * 
     * @param stamped  Stamped<Point3d> with the position information
     * @return         Point array [x,y,z] 
     */
    public static double[] stampedPoint3dToPointArray(Stamped<Point3d> stamped) {
        return  new double[] {stamped.data.x, stamped.data.y, stamped.data.z};
    }
    
    /**
     * Convert a point array [x,y,z] to a Stamped<Point3d> object
     *  
     * @param p         Point to be converted
     * @param frameID   TF frame ID 
     * @param secs      Time stamp in seconds
     * @return          Stamped<Point3d> with the position information
     */
    public static Stamped<Point3d> pointArrayToStampedPoint3d(float[] p, String frameID, double secs) {

        Stamped<Point3d> point = new Stamped<Point3d>();
        
        point.data = new Point3d();
        point.data.x = p[0];
        point.data.y = p[1];
        point.data.z = p[2];
        
        point.frameID = frameID;
        point.timeStamp = secondsToRosTime(secs);
        
        return point;
    }
    
    /**
     * Convert a Stamped<Matrix4d> object to a pose array
     * 
     * @param stamped Stamped<Matrix4d> with the pose information
     * @return Row-based array with the matrix elements
     */
    public static double[] stampedMatrix4dToPoseArray(Stamped<Matrix4d> stamped) {
        Matrix4d m = stamped.data;
               
        return new double[] {m.m00, m.m01, m.m02, m.m03, m.m10, m.m11, m.m12, m.m13,
                             m.m20, m.m21, m.m22, m.m23, m.m30, m.m31, m.m32, m.m33};
    }

    
    /**
     * Convert a pose array to a Stamped<Matrix4d> object
     * 
     * @param m       Array containing the pose matrix (row-based)
     * @param frameID TF frame ID
     * @param secs    Time stamp in seconds
     * @return        Stamped<Matrix4d> with the pose information
     */
    public static Stamped<Matrix4d> poseArrayToStampedMatrix4d(float[] m, String frameID, double secs) {
        
        Stamped<Matrix4d> mat = new Stamped<Matrix4d>();
        
        mat.data = new Matrix4d();
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                mat.data.setElement(i, j, m[4*i+j]);
        
        mat.frameID = frameID;
        mat.timeStamp = secondsToRosTime(secs);
        
        return mat;
    }
    
    /**
     * Create an identity matrix Stamped<Matrix4d>
     * 
     * @return Identity matrix
     */
    public static Stamped<Matrix4d> getStampedIdentityMatrix4d() {
        
        Stamped<Matrix4d> res = new Stamped<Matrix4d>();
        res.data = new Matrix4d();
        res.data.setIdentity();
        return res;
        
    }
    
    /**
     * Create an empty Stamped<Point3d> object (position 0,0,0)
     * 
     * @return empty Stamped<Point3d>
     */
    public static Stamped<Point3d> getStampedPoint3d() {
        
        Stamped<Point3d> res = new Stamped<Point3d>();
        res.data = new Point3d();
        return res;
        
    }
    
}
