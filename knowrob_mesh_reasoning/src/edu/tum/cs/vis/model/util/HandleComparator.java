/*******************************************************************************
 * Copyright (c) 2013 Stefan Profanter. All rights reserved. This program and the accompanying
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
 * Handle comparator used to sort list of handle annotations according their handle probability.
 * 
 * @author Stefan Profanter
 * 
 */
public class HandleComparator implements Comparator<HandleAnnotation> {
	/**
	 * default minimum radius
	 */
	public static float	DEFAULT_RADIUS_MIN	= 0.004f;	// 4 millimeters
	/**
	 * default maximum radius
	 */
	public static float	DEFAULT_RADIUS_MAX	= 0.05f;	// 80 millimeters
	/**
	 * default minimum length
	 */
	public static float	DEFAULT_LENGTH_MIN	= 0.01f;	// 1cm
	/**
	 * default maximum length
	 */
	public static float	DEFAULT_LENGTH_MAX	= 0.80f;	// 80cm

	/**
	 * Get weight for area coverage based on sigmoid function. Area coverage should be bigger than
	 * 0.5 (= 50%)
	 * 
	 * @param coverage
	 *            area coverage to calculate weight for
	 * @return weight for provided area coverage
	 */
	private static double getAreaCoverageWeight(float coverage) {
		// calculates sigmoid: 1/(1+e^(-(x-0.5)*20))
		return 1 / (1 + Math.exp(-(Math.min(coverage, 1) - 0.5) * 20)) * WEIGHT_COVERAGE;
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

	/**
	 * Calculates the weight for a handle. If no min/max radius is given (=-1) then the weight is
	 * proportional to the radius. If only radius is given, the length is ignored. If both are
	 * given, the weight is calculated as a combination of radius and length.
	 * 
	 * @param h
	 *            Handle annotation
	 * @param model
	 *            parent model of annotation (needed to get unscaled value)
	 * @param minRadius
	 *            minimum desired handle radius
	 * @param maxRadius
	 *            maximum desired handle radius
	 * @param minLength
	 *            minimum desired handle length
	 * @param maxLength
	 *            maximum desired handle length
	 * @return weight for the handle. The bigger the weight, the more probable it is a handle
	 */
	public static double getHandleWeight(HandleAnnotation h, Model model, double minRadius,
			double maxRadius, double minLength, double maxLength) {
		double w = getMinMaxWeight(model.getUnscaled(h.getCone().getRadiusAvg()), minRadius,
				maxRadius, WEIGHT_RADIUS, model.getUnscaled(h.getCone().getRadiusAvg()))
				* getMinMaxWeight(model.getUnscaled(h.getCone().getHeight()), minLength, maxLength,
						WEIGHT_LENGTH, 1) * getAreaCoverageWeight(h.getAreaCoverage());

		// it may be that the cone wasn't fit correctly and is therefore NAN
		if (Double.isNaN(w))
			w = 0;
		return w;
	}

	/**
	 * Get weight of x for specified min/max values.
	 * 
	 * @param x
	 *            value to calculate weight for
	 * @param min
	 *            min value
	 * @param max
	 *            max value
	 * @param itemWeight
	 *            basic weighting of x
	 * @param fallback
	 *            fallback weight if min and/or max have invalid values (smaller than 0)
	 * @return the weight for x
	 */
	private static double getMinMaxWeight(float x, double min, double max, double itemWeight,
			double fallback) {
		return (min >= 0 && max > min) ? getWeight(x, min, max) * itemWeight + x * 0.001 : fallback
				* itemWeight;
	}

	/**
	 * Returns the result of (input it to Wolfram Alpha to get nicer view):
	 * Piecewise[{{(exp(x*8/a)-1)/exp(8), x < a}, {1, a <= x <= b},{1/Gamma((1/(b-a))*(x+b-2a))
	 * ,x>b}}] Where a = minVal, b=maxVal
	 * 
	 * @param x
	 *            value to calculate weight for
	 * @param minVal
	 *            min value
	 * @param maxVal
	 *            max value
	 * @return the weight for x
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

	/**
	 * Current desired min radius
	 */
	private final double		minRadius;

	/**
	 * Current desired max radius
	 */
	private final double		maxRadius;

	/**
	 * Current desired min length
	 */
	private final double		minLength;

	/**
	 * Current desired max length
	 */
	private final double		maxLength;

	/**
	 * Parent model
	 */
	private final Model			model;

	/**
	 * Basic weighting factor (importance) of radius
	 */
	private final static double	WEIGHT_RADIUS			= 2.5;

	/**
	 * Basic weighting factor (importance) of length
	 */
	private final static double	WEIGHT_LENGTH			= 1;

	/**
	 * Basic weighting factor (importance) of area coverage
	 */
	private final static double	WEIGHT_COVERAGE			= 0.5;

	/**
	 * Basic weighting factor (importance) of fit error
	 */
	private final static double	WEIGHT_FIT_ERROR		= 0.75;

	/**
	 * Minimum needed weight so that it is a handle
	 */
	public final static double	MIN_WEIGHT_FOR_HANDLE	= (WEIGHT_RADIUS * WEIGHT_LENGTH * WEIGHT_FIT_ERROR) * 0.25;

	/**
	 * Initialize handle comparator with given values.
	 * 
	 * @param model
	 *            parent model
	 * @param minRadius
	 *            minimum desired radius or -1
	 * @param maxRadius
	 *            maximum desired radius or -1
	 * @param minLength
	 *            minimum desired length or -1
	 * @param maxLength
	 *            maximum desired length or -1
	 */
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
