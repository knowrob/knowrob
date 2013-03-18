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
	private static double getAreaCoverageWeight(float coverage) {
		// calculates sigmoid: 1/(1+e^(-(x-0.5)*20))
		return 1 / (1 + Math.exp(-(coverage - 0.5) * 20)) * WEIGHT_COVERAGE;
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
	private final static double	WEIGHT_RADIUS	= 2;
	private final static double	WEIGHT_LENGTH	= 1;
	private final static double	WEIGHT_COVERAGE	= 1;

	/*
	 * mesh_handle_comparator(Comp, W1, W2) :-
	jpl_call(W1,'getAreaCoverage',[],Cov1),
	jpl_call(W2,'getAreaCoverage',[],Cov2),
	jpl_call(W1,'getRadiusAvgUnscaled',[],Rad1),
	jpl_call(W2,'getRadiusAvgUnscaled',[],Rad2),
	( mesh_min_radius(MinRadius),mesh_max_radius(MaxRadius) ->
		(
			(Rad1 > MinRadius , Rad1 < MaxRadius) ->
			Rad1Ok = true
			; Rad1Ok = false
		)
		; Rad1Ok = false
	),
	( mesh_min_radius(MinRadius),mesh_max_radius(MaxRadius) ->
		(
			(Rad2 > MinRadius , Rad2 < MaxRadius) ->
			Rad2Ok = true
			; Rad2Ok = false
		)
		; Rad2Ok = false
	),
	(	Cov1 < 0.6 , Cov2 >= 0.6 -> Comp = '>'
	;	Cov1 >= 0.6, Cov2 < 0.6 -> Comp = '<'
	;	not(Rad1Ok), Rad2Ok -> Comp = '>'
	;	Rad1Ok, not(Rad2Ok) -> Comp = '<'
	;	jpl_call(W1,'getHeightUnscaled',[],H1),
		jpl_call(W2,'getHeightUnscaled',[],H2),
		(   H1 < H2 -> Comp = '>'
		;   H1 >= H2 -> Comp = '<')
	).
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
		double w1 = getRadiusWeight(model.getUnscaled(o1.getCone().getRadiusAvg()))
				* getLengthWeight(model.getUnscaled(o1.getCone().getHeight()))
				* getAreaCoverageWeight(o1.getAreaCoverage());
		double w2 = getRadiusWeight(model.getUnscaled(o2.getCone().getRadiusAvg()))
				* getLengthWeight(model.getUnscaled(o2.getCone().getHeight()))
				* getAreaCoverageWeight(o2.getAreaCoverage());
		// it may be that the cone wasn't fit correctly and is therefore NAN
		if (Double.isNaN(w1))
			w1 = 0;
		if (Double.isNaN(w2))
			w2 = 0;
		return Double.compare(w1, w2) * (-1);
	}

	private double getLengthWeight(float x) {
		return (minLength >= 0 && maxLength > minLength) ? getWeight(x, minLength, maxLength)
				* WEIGHT_LENGTH : 1;
	}

	private double getRadiusWeight(float x) {
		return (minRadius >= 0 && maxRadius > minRadius) ? getWeight(x, minRadius, maxRadius)
				* WEIGHT_RADIUS : 1;
	}
}
