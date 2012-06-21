/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.analyzer;

import java.awt.Color;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicInteger;

import org.apache.log4j.Logger;

import edu.tum.cs.vis.model.uima.annotation.primitive.PrimitiveType;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Curvature;
import edu.tum.cs.vis.model.util.ThreadPool;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * @author Stefan Profanter
 * 
 */
public class PrimitiveAnalyzer extends MeshAnalyzer {

	/**
	 * Log4J Logger
	 */
	private static Logger	logger					= Logger.getLogger(PrimitiveAnalyzer.class);

	private static float	EPSILON_SAME_CURVATURE	= 1f;
	private static float	EPSILON_IS_PLANE		= 1.7f;
	private static float	EPSILON_FLAT			= 6f;

	static void analyzeVertex(Vertex v, CopyOnWriteArrayList<Vertex> planeVertices,
			CopyOnWriteArrayList<Vertex> sphereVertices,
			CopyOnWriteArrayList<Vertex> cylinderVertices) {

		Curvature c = v.getCurvature();

		// float diff = Math.abs(c.getCurvatureMax() - c.getCurvatureMin());
		float diff = Math.abs((c.getCurvatureMax() + c.getCurvatureMin()) / 2);
		System.out.println("diff: " + diff);
		diff = 1 / diff;

		PrimitiveType t = getPrimitiveType(v);
		if (t == PrimitiveType.PLANE) {
			planeVertices.add(v);
			v.color = new Color(255, 0, 0);
		} else if (t == PrimitiveType.SPHERE) {
			sphereVertices.add(v);
			v.color = new Color(0, 255, 0);
		} else if (t == PrimitiveType.CONE) {
			cylinderVertices.add(v);
			v.color = new Color(0, 0, 255);
		}
		// v.color = Color.getHSBColor(diff, 1, 1.0f);

	}

	private static PrimitiveType getPrimitiveType(Vertex v) {
		Curvature c = v.getCurvature();
		if (Math.abs(c.getCurvatureMin()) <= EPSILON_FLAT
				|| Math.abs(c.getCurvatureMax()) <= EPSILON_FLAT) {
			// Plane or cylinder
			if (Math.abs((c.getCurvatureMax() + c.getCurvatureMin()) / 2) < EPSILON_IS_PLANE) {
				return PrimitiveType.PLANE;
			}

			return PrimitiveType.CONE;
		}
		return PrimitiveType.SPHERE;

		/*	if (Math.abs(c.getCurvatureMax() - c.getCurvatureMin()) < EPSILON_SAME_CURVATURE) {
				// it is either a plane or sphere
				if (Math.abs((c.getCurvatureMax() + c.getCurvatureMin()) / 2) < EPSILON_IS_PLANE)
					return PrimitiveType.PLANE;

				return PrimitiveType.SPHERE;
			}

			// It is a cylinder or cone
			// A Cylinder is a special form of a cone.
			return PrimitiveType.CONE;*/
	}

	ArrayList<Vertex>	allVertices;

	/**
	 * Number of triangles already elaborated/processed. Used for indicating current process
	 */
	final AtomicInteger	trianglesElaborated	= new AtomicInteger(0);

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#getLogger()
	 */
	@Override
	public Logger getLogger() {
		return logger;
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#getName()
	 */
	@Override
	public String getName() {
		return "Primitive";
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#processStart(edu.tum.cs.vis.model.uima.cas.MeshCas)
	 */
	@Override
	public void processStart(MeshCas cas) {
		allVertices = cas.getModel().getVertices();

		final CopyOnWriteArrayList<Vertex> planeVertices = new CopyOnWriteArrayList<Vertex>();
		final CopyOnWriteArrayList<Vertex> sphereVertices = new CopyOnWriteArrayList<Vertex>();
		final CopyOnWriteArrayList<Vertex> cylinderVertices = new CopyOnWriteArrayList<Vertex>();

		List<Callable<Void>> threads = new LinkedList<Callable<Void>>();

		final int interval = 500;

		for (int start = 0; start < allVertices.size(); start += interval) {
			final int st = start;
			threads.add(new Callable<Void>() {

				@Override
				public Void call() throws Exception {
					int end = Math.min(st + interval, allVertices.size());
					for (int i = st; i < end; i++) {
						analyzeVertex(allVertices.get(i), planeVertices, sphereVertices,
								cylinderVertices);
						trianglesElaborated.incrementAndGet();
					}
					return null;
				}

			});
		};

		ThreadPool.executeInPool(threads);

	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#updateProgress()
	 */
	@Override
	public void updateProgress() {
		if (allVertices != null && allVertices.size() > 0) {
			setProgress(trianglesElaborated.get() / (float) allVertices.size() * 100.0f);
		}

	}

}
