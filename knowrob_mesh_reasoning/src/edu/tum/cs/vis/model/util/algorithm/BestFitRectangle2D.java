/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 * 
 * Base implementation:
 * http://www.mathworks.com/matlabcentral/fileexchange/31126-2d-minimal-bounding-box
 * 
 ******************************************************************************/
package edu.tum.cs.vis.model.util.algorithm;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;

import javax.vecmath.Point2f;
import javax.vecmath.Vector2f;

import org.ejml.simple.SimpleMatrix;

/**
 * Best fit a rectangle into 2D points.
 * 
 * @author Stefan Profanter
 * 
 * @see <a
 *      href="http://www.profanter.me/pub/BachelorThesis/Thesis.pdf">http://www.profanter.me/pub/BachelorThesis/Thesis.pdf</a>
 *      chapter 3.3.3.1
 */
public class BestFitRectangle2D {

	/**
	 * Get best fit rectangle for given points in 2D their orthonormal basis.
	 * 
	 * @param points
	 *            points to fit rectangle into
	 * @param U
	 *            orthonormal basis for points
	 * @param V
	 *            orthonormal basis for points
	 * @return array with 4 points representing the rectangle corners
	 */
	public static Point2f[] getBestFitRectangle(Point2f points[], Vector2f U, Vector2f V) {
		GrahamScan graham = new GrahamScan(points);
		ArrayList<Point2f> CH = graham.hull();

		for (int c = 0; c < 5; c++)
			for (int i = 0; i < CH.size(); i++) {
				Point2f a = CH.get(i);
				CH.set(i, CH.get((i + CH.size() - 1) % CH.size()));
				CH.set((i + CH.size() - 1) % CH.size(), a);
			}
		// CH.add(CH.get(0));

		ArrayList<Point2f> E = new ArrayList<Point2f>(CH.size() - 1);

		// E = diff(CH, 1,2); % CH edges
		for (int i = 0; i < CH.size() - 1; i++) {
			Point2f p = (Point2f) CH.get(i + 1).clone();
			p.sub(CH.get(i));
			E.add(p);
		}
		// T = atan2(E(2,:),E(1,:)); % angle of CH edges (used for rotation)
		float[] T1 = new float[E.size()];
		for (int i = 0; i < E.size(); i++) {
			T1[i] = (float) Math.atan2(E.get(i).y, E.get(i).x);
		}

		// T = unique(mod(T,pi/2)); % reduced to the unique set of first quadrant angles
		HashSet<Float> T2 = new HashSet<Float>();
		for (int i = 0; i < T1.length; i++) {
			T2.add((float) ((T1[i] + 2 * Math.PI) % (Math.PI / 2)));
		}

		float[] T = new float[T2.size()];
		int x = 0;
		for (Float f : T2)
			T[x++] = f;

		Arrays.sort(T);

		/*
		 * % create rotation matrix which contains
		 * % the 2x2 rotation matrices for *all* angles in T
		 * % R is a 2n*2 matrix
		 * 
		 * R = cos( reshape(repmat(T,2,2),2*length(T),2) ... % duplicate angles in T
		 *     + repmat([0 -pi ; pi 0]/2,length(T),1));   % shift angle to convert sine in cosine
		 *
		 */
		SimpleMatrix R = new SimpleMatrix(T.length * 2, 2);

		for (int i = 0; i < T.length; i++) {
			R.set(i * 2, 0, Math.cos(T[i]));
			R.set(i * 2, 1, Math.cos(T[i] - Math.PI / 2));
			R.set(i * 2 + 1, 0, Math.cos(T[i] + Math.PI / 2));
			R.set(i * 2 + 1, 1, Math.cos(T[i]));
		}

		SimpleMatrix CHMat = new SimpleMatrix(2, CH.size());
		for (int i = 0; i < CH.size(); i++) {
			CHMat.set(0, i, CH.get(i).x);
			CHMat.set(1, i, CH.get(i).y);
		}

		SimpleMatrix RCH = R.mult(CHMat);

		/*
		 * % compute border size  [w1;h1;w2;h2;....;wn;hn]
		 * % and area of bounding box for all possible edges
		 * bsize = max(RCH,[],2) - min(RCH,[],2);
		 */

		float[] bsize = new float[RCH.getMatrix().numRows];

		for (int r = 0; r < RCH.getMatrix().numRows; r++) {
			float max = Float.MIN_VALUE;
			float min = Float.MAX_VALUE;

			for (int c = 0; c < RCH.getMatrix().numCols; c++) {
				min = (float) Math.min(min, RCH.get(r, c));
				max = (float) Math.max(max, RCH.get(r, c));
			}
			bsize[r] = max - min;
		}

		/*
		 * area  = prod(reshape(bsize,2,length(bsize)/2));
		 */
		float[] area = new float[bsize.length / 2];
		for (int i = 0; i < area.length; i++) {
			area[i] = bsize[i * 2] * bsize[i * 2 + 1];
		}

		/*
		 * % find minimal area, thus the index of the angle in T 
		 * [a,i] = min(area); 
		 */
		float a = area[0];
		int i = 0;
		for (int j = 1; j < area.length; j++) {
			if (area[j] < a) {
				i = j;
				a = area[j];
			}
		}

		/*
		 * % compute the bound (min and max) on the rotated frame
		 * Rf    = R(2*i+[-1 0],:);   % rotated frame
		 */
		// Matlab has start index 1, not 0
		i = (i + 1) * 2;
		SimpleMatrix Rf = new SimpleMatrix(2, 2);
		Rf.setRow(0, 0, R.get(i - 2, 0), R.get(i - 2, 1));
		Rf.setRow(1, 0, R.get(i - 1, 0), R.get(i - 1, 1));

		/*
		 * bound = Rf * CH;           % project CH on the rotated frame
		 */
		SimpleMatrix bound = Rf.mult(CHMat);

		/*
		 * bmin  = min(bound,[],2);
		 * bmax  = max(bound,[],2);
		 */

		float[] bmin = new float[2];
		bmin[0] = (float) bound.get(0, 0);
		bmin[1] = (float) bound.get(1, 0);
		float[] bmax = new float[2];
		bmax[0] = (float) bound.get(0, 0);
		bmax[1] = (float) bound.get(1, 0);
		for (int c = 1; c < bound.getMatrix().numCols; c++) {
			for (int r = 0; r < 2; r++) {
				bmin[r] = (float) Math.min(bmin[r], bound.get(r, c));
				bmax[r] = (float) Math.max(bmax[r], bound.get(r, c));
			}
		}

		/*
		 * % compute the corner of the bounding box
		 * Rf = Rf';
		 */
		SimpleMatrix RfTrans = Rf.transpose();
		Point2f[] ret = new Point2f[4];
		// bb(:,4) = bmax(1)*Rf(:,1) + bmin(2)*Rf(:,2);
		ret[3] = new Point2f((float) (bmax[0] * RfTrans.get(0, 0) + bmin[1] * RfTrans.get(0, 1)),
				(float) (bmax[0] * RfTrans.get(1, 0) + bmin[1] * RfTrans.get(1, 1)));
		// bb(:,1) = bmin(1)*Rf(:,1) + bmin(2)*Rf(:,2);
		ret[0] = new Point2f((float) (bmin[0] * RfTrans.get(0, 0) + bmin[1] * RfTrans.get(0, 1)),
				(float) (bmin[0] * RfTrans.get(1, 0) + bmin[1] * RfTrans.get(1, 1)));
		// bb(:,2) = bmin(1)*Rf(:,1) + bmax(2)*Rf(:,2);
		ret[1] = new Point2f((float) (bmin[0] * RfTrans.get(0, 0) + bmax[1] * RfTrans.get(0, 1)),
				(float) (bmin[0] * RfTrans.get(1, 0) + bmax[1] * RfTrans.get(1, 1)));
		// bb(:,3) = bmax(1)*Rf(:,1) + bmax(2)*Rf(:,2);
		ret[2] = new Point2f((float) (bmax[0] * RfTrans.get(0, 0) + bmax[1] * RfTrans.get(0, 1)),
				(float) (bmax[0] * RfTrans.get(1, 0) + bmax[1] * RfTrans.get(1, 1)));

		if (U != null) {
			U.x = (float) Rf.get(0, 0);
			U.y = (float) Rf.get(0, 1);
		}
		if (V != null) {
			V.x = (float) Rf.get(1, 0);
			V.y = (float) Rf.get(1, 1);
		}

		return ret;
	}
}
