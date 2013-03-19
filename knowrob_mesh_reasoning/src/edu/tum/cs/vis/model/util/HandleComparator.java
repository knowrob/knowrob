/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2013
 ******************************************************************************/
package edu.tum.cs.vis.model.util;

import java.util.Comparator;

import org.apache.commons.math3.special.Gamma;

import edu.tum.cs.vis.model.Model;
import edu.tum.cs.vis.model.uima.annotation.HandleAnnotation;

/**
 * @author Stefan Profanter
 * 
 */
public class HandleComparator implements Comparator<HandleAnnotation> {
	public static float	DEFAULT_RADIUS_MIN	= 0.004f;	// 4 millimeters
	public static float	DEFAULT_RADIUS_MAX	= 0.05f;	// 80 millimeters
	public static float	DEFAULT_LENGTH_MIN	= 0.01f;	// 1cm
	public static float	DEFAULT_LENGTH_MAX	= 0.80f;	// 80cm

	private static double getAreaCoverageWeight(float coverage) {
		// calculates sigmoid: 1/(1+e^(-(x-0.5)*20))
		return 1 / (1 + Math.exp(-(Math.min(coverage, 1) - 0.5) * 20)) * WEIGHT_COVERAGE;
	}

	private static double getFitErrorWeight(float fitError) {
		// calculates mirrored sigmoid: 1-1/(1+e^(-(x-0.5)*10))

		return 1 - 1 / (1 + Math.exp(-(fitError - 0.5) * 10)) * WEIGHT_FIT_ERROR;
	}

	/*public static void main(String[] args) {
		double x = 0;
		double min = 0.05;
		double max = 0.12;

		try {
			// Create file
			FileWriter fstream = new FileWriter("/home/stefan/points.txt");
			BufferedWriter out = new BufferedWriter(fstream);
			double w = 0;
			while (x < max * 3) {
				w = getWeight(x, min, max);
				out.write(x + " " + w + "\n");
				x += (max - min) / 100;
			}
			out.close();
		} catch (Exception e) {// Catch exception if any
			System.err.println("Error: " + e.getMessage());
		}

	}*/

	public static double getHandleWeight(HandleAnnotation o1, Model model, double minRadius,
			double maxRadius, double minLength, double maxLength) {
		double w = getMinMaxWeight(model.getUnscaled(o1.getCone().getRadiusAvg()), minRadius,
				maxRadius, WEIGHT_RADIUS)
				* getMinMaxWeight(model.getUnscaled(o1.getCone().getHeight()), minLength,
						maxLength, WEIGHT_LENGTH) * getAreaCoverageWeight(o1.getAreaCoverage());

		// it may be that the cone wasn't fit correctly and is therefore NAN
		if (Double.isNaN(w))
			w = 0;
		return w;
	}

	private static double getMinMaxWeight(float x, double min, double max, double itemWeight) {
		return (min >= 0 && max > min) ? getWeight(x, min, max) * itemWeight : 1;
	}

	/**
	 * Returns the result of (input it to Wolfram Alpha to get nicer view):
	 * Piecewise[{{(exp(x*8/a)-1)/exp(8), x < a}, {1, a <= x <= b},{1/Gamma((1/(b-a))*(x+b-2a))
	 * ,x>b}}] Where a = minVal, b=maxVal
	 * 
	 * @param x
	 * @param minVal
	 * @param maxVal
	 * @return
	 */
	private static double getWeight(double x, double minVal, double maxVal) {
		if (x == 0)
			return minVal > 0 ? 0 : 1;
		if (x < minVal)
			return (Math.exp(x * 8 / minVal) - 1) / Math.exp(8);
		else if (x > maxVal)
			return 1 / Gamma.gamma((1 / (maxVal - minVal)) * (x + maxVal - 2 * minVal));
		else
			return 1;
	}

	private final double		minRadius;

	private final double		maxRadius;

	private final double		minLength;
	private final double		maxLength;
	private final Model			model;
	private final static double	WEIGHT_RADIUS		= 2;

	private final static double	WEIGHT_LENGTH		= 1;

	private final static double	WEIGHT_COVERAGE		= 0.5;
	private final static double	WEIGHT_FIT_ERROR	= 1;

	public HandleComparator(Model model, double minRadius, double maxRadius, double minLength,
			double maxLength) {
		this.model = model;
		this.minRadius = minRadius;
		this.maxRadius = maxRadius;
		this.minLength = minLength;
		this.maxLength = maxLength;
	}

	/* (non-Javadoc)
	 * @see java.util.Comparator#compare(java.lang.Object, java.lang.Object)
	 */
	@Override
	public int compare(HandleAnnotation o1, HandleAnnotation o2) {
		double w1 = getHandleWeight(o1, model, minRadius, maxRadius, minLength, maxLength);
		double w2 = getHandleWeight(o2, model, minRadius, maxRadius, minLength, maxLength);
		return Double.compare(w1, w2) * (-1);
	}

}
