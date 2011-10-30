package edu.tum.cs.vis.model.util;

import javax.vecmath.Point3f;

/**
 * Base class for all drawable model parts (Line / Triangle)
 * 
 * @author Stefan Profanter
 *
 */
public abstract class DrawObject {
	/**
	 * the position points of the object
	 */
	public Point3f position[];

	/**
	 * Moves the triangle in the given direction.
	 * 
	 * @param x
	 *            direction in x
	 * @param y
	 *            direction in y
	 * @param z
	 *            direction in z
	 */
	public void translate(float x, float y, float z) {
		for (int v = 0; v < position.length; v++) {
			position[v].x += x;
			position[v].y += y;
			position[v].z += z;
		}
	}

	/**
	 * Scales all coordinates of the position points by the given factor
	 * 
	 * @param factor
	 *            The Scale factor
	 */
	public void scale(float factor) {
		for (int v = 0; v < position.length; v++) {
			position[v].x *= factor;
			position[v].y *= factor;
			position[v].z *= factor;
		}
	}

	/**
	 * Apply 4x4 transformation matrix to the position vectors of this object
	 * @param matrix the transformation matrix
	 */
	public void transform(float[][] matrix) {
		for (int v = 0; v < position.length; v++) {
			float[] newPos = new float[4];
			for (int row = 0; row < 4; row++) {
				newPos[row] = position[v].x * matrix[row][0] + position[v].y
						* matrix[row][1] + position[v].z * matrix[row][2]
						+ matrix[row][3];
			}
			position[v].x = newPos[0] / newPos[3];
			position[v].y = newPos[1] / newPos[3];
			position[v].z = newPos[2] / newPos[3];
		}
	}

	/**
	 * Multiplies the two given matrix. Must have correct size for multiplying.
	 * @param mat1 matrix 1
	 * @param mat2 matrix 2
	 */
	public static float[][] MatrixMultiply(float mat1[][], float mat2[][])
	{
		int x = mat1.length;
		int y = mat2.length;
		float result[][] = new float[x][y];

		for (int i = 0; i < x; i++) {
			for (int j = 0; j < y - 1; j++) {
				for (int k = 0; k < y; k++) {

					result[i][j] += mat1[i][k] * mat2[k][j];
				}
			}
		}
		
		return result;
	}
}
