/*
 * Copyright (c) 2012-14 Moritz Tenorth
 *
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
 *
*/

package org.knowrob.utils;

import java.util.LinkedList;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

public class MathUtil {
	/**
	 * Computes quaternion from 4x4 matrix
	 * @param matrix The 4x4 matrix
	 * @return Array that represents the quaternion
	 */
	public static double[] matrixToQuaternion(float[] matrix) {
		double[] p = new double[16];
		for(int i=0;i<16;i++) {
			p[i] = matrix[i];
		}
		return matrixToQuaternion(p);
	}

	/**
	 * Computes quaternion from 4x4 matrix
	 * @param matrix The 4x4 matrix
	 * @return Array that represents the quaternion
	 */
	public static double[] matrixToQuaternion(double[] matrix) {
		Matrix4d poseMat = new Matrix4d(matrix);
		Quat4d quat = new Quat4d();
		quat.set(poseMat);
		return new double[] { quat.x, quat.y, quat.z, quat.w };
	}
	public static double[] matrixToQuaternion(Matrix4d poseMat) {
		Quat4d quat = new Quat4d();
		quat.set(poseMat);
		return new double[] { quat.x, quat.y, quat.z, quat.w };
	}
	public static double[] matrixToQuaternion(Matrix3d poseMat) {
		Quat4d quat = new Quat4d();
		quat.set(poseMat);
		return new double[] { quat.x, quat.y, quat.z, quat.w };
	}

	public static double[] matrix(float[] translation, float[] quatenrion) {
		return matrix(
				new double[] {translation[0], translation[1], translation[2]},
				new double[] {quatenrion[0], quatenrion[1], quatenrion[2], quatenrion[3]});
	}

	public static double[] matrix(double[] translation, double[] quatenrion) {
		Quat4d q = new Quat4d(quatenrion[0], quatenrion[1], quatenrion[2], quatenrion[3]);
		Vector3d t = new Vector3d(translation);
		Matrix4d poseMat = new Matrix4d(q, t, 1.0);
		return new double[] {
				poseMat.m00, poseMat.m01, poseMat.m02, poseMat.m03,
				poseMat.m10, poseMat.m11, poseMat.m12, poseMat.m13,
				poseMat.m20, poseMat.m21, poseMat.m22, poseMat.m23,
				poseMat.m20, poseMat.m31, poseMat.m32, poseMat.m33
		};
	}
	
	public static Quat4d quaternion(double[] q) {
		return new Quat4d(q[0], q[1], q[2], q[3]);
	}
	public static Quat4d quaternion(float[] q) {
		return new Quat4d(q[0], q[1], q[2], q[3]);
	}
	
	public static Matrix3d quaternionMatrix(double[] q) {
		return quaternionMatrix(quaternion(q));
	}
	public static Matrix3d quaternionMatrix(Quat4d q) {
		Matrix3d rot = new Matrix3d();
		rot.set(q);
		return rot;
	}
	
	public static double[] quaternionInverse(float[] q) {
		return quaternionInverse(quaternion(q));
	}
	public static double[] quaternionInverse(double[] q) {
		return quaternionInverse(quaternion(q));
	}
	public static double[] quaternionInverse(Quat4d q) {
		Quat4d inv = new Quat4d(q);
		inv.inverse();
		return new double[] { inv.x, inv.y, inv.z, inv.w };
	}
	
	public static double[] quaternionTransform(float[] q, float[] t) {
		return quaternionTransform(quaternion(q), t);
	}
	public static double[] quaternionTransform(double[] q, float[] t) {
		return quaternionTransform(quaternion(q), t);
	}
	public static double[] quaternionTransform(double[] q, double[] t) {
		return quaternionTransform(quaternion(q), t);
	}
	public static double[] quaternionTransform(Quat4d q, float[] t) {
		Matrix3d mat = quaternionMatrix(q);
		Vector3d vec = new Vector3d(t[0], t[1], t[2]);
		mat.transform(vec);
		return new double[] { vec.x, vec.y, vec.z };
	}
	public static double[] quaternionTransform(Quat4d q, double[] t) {
		Matrix3d mat = quaternionMatrix(q);
		Vector3d vec = new Vector3d(t[0], t[1], t[2]);
		mat.transform(vec);
		return new double[] { vec.x, vec.y, vec.z };
	}

	public static float[] quaternionDifference(float[] q0, float[] q1) {
		Quat4d diff = quaternionDifference(
				new Quat4d(q0[0], q0[1], q0[2], q0[3]),
				new Quat4d(q1[0], q1[1], q1[2], q1[3])
		);
		return new float[] { (float)diff.x, (float)diff.y, (float)diff.z, (float)diff.w };
	}

	public static double[] quaternionDifference(double[] q0, double[] q1) {
		Quat4d diff = quaternionDifference(
				new Quat4d(q0[0], q0[1], q0[2], q0[3]),
				new Quat4d(q1[0], q1[1], q1[2], q1[3])
		);
		return new double[] { diff.x, diff.y, diff.z, diff.w };
	}

	public static Quat4d quaternionDifference(Quat4d q0, Quat4d q1) {
		Quat4d inv = new Quat4d(q0);
		inv.inverse();
		Quat4d diff = new Quat4d(inv);
		diff.mul(q1);
		return diff;
	}

	public static float[] quaternionMultiply(float[] q0, float[] q1) {
		Quat4d diff = quaternionMultiply(
				new Quat4d(q0[0], q0[1], q0[2], q0[3]),
				new Quat4d(q1[0], q1[1], q1[2], q1[3])
		);
		return new float[] { (float)diff.x, (float)diff.y, (float)diff.z, (float)diff.w };
	}

	public static double[] quaternionMultiply(double[] q0, double[] q1) {
		Quat4d diff = quaternionMultiply(
				new Quat4d(q0[0], q0[1], q0[2], q0[3]),
				new Quat4d(q1[0], q1[1], q1[2], q1[3])
		);
		return new double[] { diff.x, diff.y, diff.z, diff.w };
	}

	public static Quat4d quaternionMultiply(Quat4d q0, Quat4d q1) {
		Quat4d mul = new Quat4d();
		mul.mul(q0, q1);
		return mul;
	}

	public static double[] orientationToQuaternion(float[] dir) {
		return orientationToQuaternion(new Vector3d(dir[0], dir[1], dir[2]));
	}

	public static double[] orientationToQuaternion(double[] dir) {
		return orientationToQuaternion(new Vector3d(dir[0], dir[1], dir[2]));
	}

	public static double[] orientationToQuaternion(Vector3d dir) {
		Vector3d up = new Vector3d(0.0,0.0,1.0);
		
		// Rotation Axis and angle that rotates `dir` to the `up` vector.
		Vector3d axis = new Vector3d();
		axis.cross(dir, up);
		double angle = dir.angle(up);
		
		Quat4d q = new Quat4d();
		Matrix3d mat = new Matrix3d();
		mat.set(new AxisAngle4d(axis, angle));
		q.set(mat);
		return new double[] { q.x, q.y, q.z, q.w };
	}
	
	/**
	 * Read double array from input string.
	 * @param in The input string
	 * @param separator Separator character of numbers in input string
	 * @return The vector represented as double array
	 */
	public static double[] parseVector(String in, String separator) {
		try {
			String[] elements = in.split(separator);
			LinkedList<Double> outList = new LinkedList<Double>();
			for(int i=0; i<elements.length; ++i) {
				String elem = elements[i].trim();
				if(!elem.isEmpty()) {
					outList.add(Double.valueOf(elem));
				}
			}
			double[] p = new double[outList.size()];
			for(int i=0; i<outList.size(); ++i) {
				p[i] = outList.get(i);
			}
			return p;
		}
		catch (Exception e) {
			System.err.println("Failed to parse vector: " + e.getMessage());
			e.printStackTrace();
			return null;
		}
	}
	
	public static void main(String[] args) {
		Vector3d dir = new Vector3d(1.0, 2.0, 3.0);
		double[] q = orientationToQuaternion(dir);
		System.out.println("q: " + q[0] + " " + q[1] + " " + q[2] + " " + q[3]);
	}
}
