/*******************************************************************************
 * Copyright (c) 2014 Andrei Stoica. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Andrei Stoica - initial API and implementation, Year: 2014
 ******************************************************************************/

package edu.tum.cs.vis.model.util;

/**
 * Class that defines all values that can tweak the performance of the mesh_reasoning algorithms.
 * 
 * @author Andrei Stoica
 * 
 */
public class UtilityValues {
	/**
	 * Distance tolerance which classifies metric objects as equal
	 */
	public static final float	DISTANCE_TOL					= 1e-5f;

	/**
	 * Parameter used as a minimum boundary of distance measures calculations
	 */
	public static final float	EPSILON							= 1e-5f;

	/**
	 * Number of clusters used to classify the vertices of the model according to their curvatures
	 */
	public static final int		NUM_CLUSTERS					= 30;

	/**
	 * Maximum number of iterations after which the KMeans algorithm stops
	 */
	public static final int		ITERATIONS_LIM					= 180;

	/**
	 * Maximum number of iterations after which the region merging stops
	 */
	public static final int		ITERATIONS_LIM_REGION_MERGING	= 300;

	/**
	 * Area filtering threshold
	 */
	public static final float	AREA_LIM						= 5e-2f;

	/**
	 * Minimum distance threshold for the Region Adjacency Graph
	 */
	public static final float	MIN_DISTANCE_TOL				= 7.5f;

	/**
	 * Angle tolerance (in DEG) for deciding whether or not two lines are parallel
	 */
	public static final float	ANGLE_TOLERANCE					= 0.1f;

	/**
	 * Curvature hue / saturation smoothing value
	 */
	public static final float	CURV_SMOOTHING					= 0.01f;

	/**
	 * Sharp edge threshold angle value in degrees
	 */
	public static final float	SHARP_EDGE_ANGLE_TOL			= 65.0f;

	/**
	 * No instantiation
	 */
	private UtilityValues() {
		return;
	}
}