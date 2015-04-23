package org.knowrob.utils;

import java.util.LinkedList;

import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;

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
		
		return new double[] { quat.w, quat.x, quat.y, quat.z };
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
}
