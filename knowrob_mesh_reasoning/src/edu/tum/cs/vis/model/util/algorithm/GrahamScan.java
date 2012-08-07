/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - ported to mesh reasoning, Year: 2012
 * 
 * Original Source: http://algs4.cs.princeton.edu/99hull/
 * 
 ******************************************************************************/
package edu.tum.cs.vis.model.util.algorithm;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Stack;

import javax.vecmath.Point2f;

/**
 * Create points from standard input and compute the convex hull using Graham scan algorithm.
 * 
 * May be floating-point issues if x- and y-coordinates are not integers.
 * 
 * Source: http://algs4.cs.princeton.edu/99hull/
 * 
 * @author Stefan Profanter
 * 
 */
public class GrahamScan {
	/**
	 * compare other points relative to polar angle (between 0 and 2pi) they make with this Point
	 **/
	private class PolarOrder implements Comparator<Point2f> {
		/**
		 * reference point
		 */
		private final Point2f	refPoint;

		/**
		 * default constructor
		 * 
		 * @param refPoint
		 *            ref point data
		 */
		public PolarOrder(Point2f refPoint) {
			this.refPoint = refPoint;
		}

		@Override
		public int compare(Point2f q1, Point2f q2) {
			double dx1 = q1.x - refPoint.x;
			double dy1 = q1.y - refPoint.y;
			double dx2 = q2.x - refPoint.x;
			double dy2 = q2.y - refPoint.y;

			if (dy1 >= 0 && dy2 < 0)
				return -1; // q1 above; q2 below
			else if (dy2 >= 0 && dy1 < 0)
				return +1; // q1 below; q2 above
			else if (dy1 == 0 && dy2 == 0) { // 3-collinear and horizontal
				if (dx1 >= 0 && dx2 < 0)
					return -1;
				else if (dx2 >= 0 && dx1 < 0)
					return +1;
				else
					return 0;
			} else
				return -ccw(refPoint, q1, q2); // both above or below

			// Note: ccw() recomputes dx1, dy1, dx2, and dy2
		}
	}

	/** compare other points relative to polar angle (between 0 and 2pi) they make with this Point */
	private class YOrder implements Comparator<Point2f> {

		/**
		 * Default constructor
		 */
		public YOrder() {}

		@Override
		public int compare(Point2f q1, Point2f q2) {
			if (q1.y < q2.y)
				return -1;
			else if (q1.y > q2.y)
				return 1;
			return 0;
		}
	}

	/**
	 * is a->b->c a counter-clockwise turn?
	 * 
	 * @param a
	 *            point a
	 * @param b
	 *            point b
	 * @param c
	 *            point c
	 * @return -1 if clockwise, +1 if counter-clockwise, 0 if collinear
	 **/
	public static int ccw(Point2f a, Point2f b, Point2f c) {
		double area2 = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
		if (area2 < 0)
			return -1;
		else if (area2 > 0)
			return +1;
		else
			return 0;
	}

	/**
	 * resulting convex hull
	 */
	private final Stack<Point2f>	hull	= new Stack<Point2f>();

	/**
	 * Main graham scan algorithm
	 * 
	 * @param pts
	 *            base points for graham scan
	 */
	public GrahamScan(Point2f[] pts) {

		// defensive copy
		int N = pts.length;
		Point2f[] points = new Point2f[N];
		for (int i = 0; i < N; i++)
			points[i] = pts[i];

		// preprocess so that points[0] has lowest y-coordinate; break ties by x-coordinate
		// points[0] is an extreme point of the convex hull
		// (alternatively, could do easily in linear time)
		Arrays.sort(points, new YOrder());

		// sort by polar angle with respect to base point points[0],
		// breaking ties by distance to points[0]
		Arrays.sort(points, 1, N, new PolarOrder(points[0]));

		hull.push(points[0]); // p[0] is first extreme point

		// find index k1 of first point not equal to points[0]
		int k1;
		for (k1 = 1; k1 < N; k1++)
			if (!points[0].equals(points[k1]))
				break;
		if (k1 == N)
			return; // all points equal

		// find index k2 of first point not collinear with points[0] and points[k1]
		int k2;
		for (k2 = k1 + 1; k2 < N; k2++)
			if (ccw(points[0], points[k1], points[k2]) != 0)
				break;
		hull.push(points[k2 - 1]); // points[k2-1] is second extreme point

		// Graham scan; note that points[N-1] is extreme point different from points[0]
		for (int i = k2; i < N; i++) {
			Point2f top = hull.pop();
			while (hull.size() > 0 && ccw(hull.peek(), top, points[i]) <= 0) {
				top = hull.pop();
			}
			hull.push(top);
			hull.push(points[i]);
		}

		assert isConvex();
	}

	/**
	 * return extreme points on convex hull in counterclockwise order as an Iterable
	 * 
	 * @return extreme points
	 */
	public ArrayList<Point2f> hull() {
		ArrayList<Point2f> s = new ArrayList<Point2f>();
		for (Point2f p : hull)
			s.add(p);
		return s;
	}

	/**
	 * check that boundary of hull is strictly convex
	 * 
	 * @return true if hull has more than 2 points
	 */
	private boolean isConvex() {
		int N = hull.size();
		if (N <= 2)
			return true;

		Point2f[] points = new Point2f[N];
		int n = 0;
		for (Point2f p : hull()) {
			points[n++] = p;
		}

		for (int i = 0; i < N; i++) {
			if (ccw(points[i], points[(i + 1) % N], points[(i + 2) % N]) <= 0) {
				return false;
			}
		}
		return true;
	}

}
